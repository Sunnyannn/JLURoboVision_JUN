// Copyright 2022 Chen Jun
#include "armor_tracker/tracker_node.hpp"

#include "armor_tracker/trajectory.h"

// STD
#include <memory>
#include <vector>

namespace rm_auto_aim
{
ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions & options)
: Node("armor_tracker", options)
{
  RCLCPP_INFO(this->get_logger(), "Starting TrackerNode!");

  yaw_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/robo_yaw", 10,std::bind(&ArmorTrackerNode::yawCallback, this, std::placeholders::_1));
  
  sentry_decision_sub_ = this->create_subscription<std_msgs::msg::Int8>(
    "/sentry_decision", 10,std::bind(&ArmorTrackerNode::sentryCallback,this,std::placeholders::_1));

  // RCLCPP_INFO(this->get_logger(), "Getting Robo_yaw ");
  // Maximum allowable armor distance in the XOY plane
  max_armor_distance_ = this->declare_parameter("max_armor_distance", 10.0);

  // Tracker
  spinning_diff = this->declare_parameter("tracker.spinning_diff",0.2);
  pitch_diff = this->declare_parameter("tracker.pitch_diff",0.0);
  yaw_diff = this->declare_parameter("tracker.yaw_diff",0.0);
  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.15);
  double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
  tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
  tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);

  // EKF
  // xa = x_armor, xc = x_robot_center
  // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
  // measurement: xa, ya, za, yaw
  // f - Process function
  auto f = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(1) * dt_;
    x_new(2) += x(3) * dt_;
    x_new(4) += x(5) * dt_;
    x_new(6) += x(7) * dt_;
    return x_new;
  };
  // J_f - Jacobian of process function
  auto j_f = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd f(9, 9);
    // clang-format off
    f <<  1,   dt_, 0,   0,   0,   0,   0,   0,   0,
          0,   1,   0,   0,   0,   0,   0,   0,   0,
          0,   0,   1,   dt_, 0,   0,   0,   0,   0, 
          0,   0,   0,   1,   0,   0,   0,   0,   0,
          0,   0,   0,   0,   1,   dt_, 0,   0,   0,
          0,   0,   0,   0,   0,   1,   0,   0,   0,
          0,   0,   0,   0,   0,   0,   1,   dt_, 0,
          0,   0,   0,   0,   0,   0,   0,   1,   0,
          0,   0,   0,   0,   0,   0,   0,   0,   1;
    // clang-format on
    return f;
  };
  // h - Observation function
  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(4);
    double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
    z(0) = xc - r * cos(yaw);  // xa
    z(1) = yc - r * sin(yaw);  // ya
    z(2) = x(4);               // za
    z(3) = x(6);               // yaw
    return z;
  };
  // J_h - Jacobian of observation function
  auto j_h = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd h(4, 9);
    double yaw = x(6), r = x(8);
    // clang-format off
    //    xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
    h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw), 0,   -cos(yaw),
          0,   0,   1,   0,   0,   0,   -r*cos(yaw),0,   -sin(yaw),
          0,   0,   0,   0,   1,   0,   0,          0,   0,
          0,   0,   0,   0,   0,   0,   1,          0,   0;
    // clang-format on
    return h;
  };
  // update_Q - process noise covariance matrix
  s2qxyz_ = declare_parameter("ekf.sigma2_q_xyz", 20.0);
  s2qyaw_ = declare_parameter("ekf.sigma2_q_yaw", 100.0);
  s2qr_ = declare_parameter("ekf.sigma2_q_r", 800.0);
  auto u_q = [this]() {
    Eigen::MatrixXd q(9, 9);
    double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
    double q_r = pow(t, 4) / 4 * r;
    // clang-format off
    //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
          0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
          0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
          0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
          0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
          0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
          0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
          0,      0,      0,      0,      0,      0,      0,      0,      q_r;
    // clang-format on
    return q;
  };
  // update_R - measurement noise covariance matrix
  r_xyz_factor = declare_parameter("ekf.r_xyz_factor", 0.05);
  r_yaw = declare_parameter("ekf.r_yaw", 0.02);
  auto u_r = [this](const Eigen::VectorXd & z) {
    Eigen::DiagonalMatrix<double, 4> r;
    double x = r_xyz_factor;
    r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
    return r;
  };
  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 9> p0;
  p0.setIdentity();
  tracker_->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

  // Reset tracker service
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;
  reset_tracker_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/tracker/reset", [this](
                        const std_srvs::srv::Trigger::Request::SharedPtr,
                        std_srvs::srv::Trigger::Response::SharedPtr response) {
      tracker_->tracker_state = Tracker::LOST;
      response->success = true;
      RCLCPP_INFO(this->get_logger(), "Tracker reset!");
      return;
    });

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  armors_sub_.subscribe(this, "/detector/armors", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_filter>(
    armors_sub_, *tf2_buffer_, target_frame_, 10, this->get_node_logging_interface(),
    this->get_node_clock_interface(), std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when transforms are available
  tf2_filter_->registerCallback(&ArmorTrackerNode::armorsCallback, this);

  // Measurement publisher (for debug usage)
  info_pub_ = this->create_publisher<auto_aim_interfaces::msg::TrackerInfo>("/tracker/info", 10);

  // Publisher
  target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS());
  
  //////
  test_pub_ = this->create_publisher<std_msgs::msg::Float64>("/test", 10);


  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armor_marker_.ns = "armors";
  armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armor_marker_.scale.x = 0.03;
  armor_marker_.scale.z = 0.125;
  armor_marker_.color.a = 1.0;
  armor_marker_.color.r = 1.0;
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tracker/marker", 10);
}

void ArmorTrackerNode::yawCallback(const std_msgs::msg::Float64 msg)
{
  robo_yaw =msg.data;
}
void ArmorTrackerNode::sentryCallback(const std_msgs::msg::Int8 msg)
{
  sentry_decision = msg.data;
}

void ArmorTrackerNode::armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_msg)
{
  // Tranform armor position from image frame to world coordinate
  for (auto & armor : armors_msg->armors) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;
    try {
      armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } catch (const tf2::ExtrapolationException & ex) {
      RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
      return;
    }
  }

  // Filter abnormal armors
  armors_msg->armors.erase(
    std::remove_if(
      armors_msg->armors.begin(), armors_msg->armors.end(),
      [this](const auto_aim_interfaces::msg::Armor & armor) {
        return abs(armor.pose.position.z) > 1.2 ||
               Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
                 max_armor_distance_;
      }),
    armors_msg->armors.end());
  // Filter sentry armors depend on actual situation 
  if( sentry_decision == 0){
     armors_msg->armors.erase(
    std::remove_if(
      armors_msg->armors.begin(), armors_msg->armors.end(),
      [this](const auto_aim_interfaces::msg::Armor & armor) {
        return armor.number == "guard";
      }),
    armors_msg->armors.end()); 
  }
  
  //  for (const auto & armor : armors_msg->armors){
  //     std::cout<<"armor.number--"<<armor.number<<std::endl;
  //   } 

//Filter low level armor
  const static std::map<std::string, int> priority_map{
    {"",4}, {"outpost", 3},{"1", 2},{"2", 3},{"3", 2},{"4", 2},{"5", 2},{"guard", 2},{"base", 2}
  };
  
  if(tracker_->tracker_state == Tracker::LOST){
    highest_level = 3;
  }
  //检查装甲板优先级是否改变
  for (const auto & armor : armors_msg->armors){
    if(priority_map.at(armor.number) < highest_level){
      highest_level = priority_map.at(armor.number);
    } 
  }
  //抹去低优先级的装甲板
   armors_msg->armors.erase(
    std::remove_if(
      armors_msg->armors.begin(), armors_msg->armors.end(),
      [this](const auto_aim_interfaces::msg::Armor & armor) {
        return priority_map.at(armor.number) > highest_level ;
      }),
    armors_msg->armors.end()); 
  // for (const auto & armor : armors_msg->armors){
  //     std::cout<<"armor.number--2"<<armor.number<<std::endl;
  //   } 

  // Init message
  auto_aim_interfaces::msg::TrackerInfo info_msg;
  auto_aim_interfaces::msg::Target target_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;

  // Update tracker
  if (tracker_->tracker_state == Tracker::LOST || priority_level != highest_level) {
    tracker_->init(armors_msg);
    target_msg.tracking = false;
    priority_level = highest_level;
  } else {
    dt_ = (time - last_time_).seconds();
    tracker_->lost_thres = static_cast<int>(lost_time_thres_ / dt_);
    tracker_->update(armors_msg);

    // Publish Info
    info_msg.position_diff = tracker_->info_position_diff;
    info_msg.yaw_diff = tracker_->info_yaw_diff;
    info_msg.position.x = tracker_->measurement(0);
    info_msg.position.y = tracker_->measurement(1);
    info_msg.position.z = tracker_->measurement(2);
    info_msg.yaw = tracker_->measurement(3);
    info_pub_->publish(info_msg);

    if (tracker_->tracker_state == Tracker::DETECTING) {
      target_msg.tracking = false;
    } else if (
      tracker_->tracker_state == Tracker::TRACKING ||
      tracker_->tracker_state == Tracker::TEMP_LOST) {
      target_msg.tracking = true;
      // Fill target message
      const auto & state = tracker_->target_state;
      target_msg.id = tracker_->tracked_id;
      target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
      target_msg.position.x = state(0);
      target_msg.velocity.x = state(1);
      target_msg.position.y = state(2);
      target_msg.velocity.y = state(3);
      target_msg.position.z = state(4);
      target_msg.velocity.z = state(5);
      target_msg.yaw = state(6);
      target_msg.v_yaw = state(7);
      target_msg.radius_1 = state(8);
      target_msg.radius_2 = tracker_->another_r;
      target_msg.dz = tracker_->dz;
      //数据提取
      pub_str target_pub;
      // target_pub.get_position_x= state(0);
      // target_pub.get_position_y= state(2);
      // target_pub.get_position_z= state(4);
      target_pub.get_velocity_x= state(1);
      target_pub.get_velocity_y= state(3);
      target_pub.get_velocity_z= state(5);
      float get_yaw = state(6);
      target_pub.get_v_yaw = state(7);
      target_pub.get_r1 = state(8);
      // 平移的开火逻辑
      // RCLCPP_INFO(this->get_logger(),"orientation_yaw_diff:%lf",tracker_->info_yaw_diff);
      float v = std::sqrt((target_pub.get_velocity_x)*(target_pub.get_velocity_x)+(target_pub.get_velocity_y)*(target_pub.get_velocity_y)+(target_pub.get_velocity_z)*(target_pub.get_velocity_z) );
        if(fabsf(target_pub.get_v_yaw) < 3.0&& v > spinning_diff && (!armors_msg->armors.empty()) )
        {
            int target_id = 0;
            double min_distance = 0;
            int count_armors = 0;
            for (const auto & armor : armors_msg->armors) {
              // Only consider armors with the same id
              if (armor.number == tracker_->tracked_id) {
              // Calculate the difference between the predicted position and the current armor position
              auto p = armor.pose.position;
              Eigen::Vector3d position_vec(p.x, p.y, p.z);
              double position_norm = position_vec.norm();
              if(position_norm < min_distance){
                  min_distance = position_norm;
                  target_id = count_armors;
                }
              count_armors++;
              }
            }
            auto armor_test = armors_msg -> armors[target_id];
            auto p = armor_test.pose.position;
            std::cout<<"i am in here ya:"<<target_id<<std::endl;
            target_pub.get_position_x= p.x;
            target_pub.get_position_y= p.y;
            target_pub.get_position_z= p.z;          
        }else{
            target_pub.get_position_x= state(0);
            target_pub.get_position_y= state(2);
            target_pub.get_position_z= state(4);
        }

      armors_num = static_cast<int>(tracker_->tracked_armors_num);
      // // 提取装甲板id
      // const static std::map<std::string,int> id_int_map{
      //   {"",0}, {"outpost",0},{"1",1},{"2",2},{"3",3},{"4",4},{"5",5},{"guard",6},{"base",7}
      // };
      // int target_id = id_int_map.at(target_msg.id);
      
      PredictPitchXY &trajectory = PredictPitchXY::getinstance();
      trajectory.GimbalControlTransform(target_pub.get_position_x ,target_pub.get_position_y, target_pub.get_position_z,
                            target_pub.get_velocity_x, target_pub.get_velocity_y, target_pub.get_velocity_z,target_pub.get_v_yaw ,
                            target_pub.get_r1, tracker_->another_r, tracker_->dz,
                            armors_num,get_yaw,target_pub.aim_x,target_pub.aim_y,target_pub.aim_z,target_pub.fire,robo_yaw,v
                            );
      
      
      float distance = std::sqrt((*(target_pub.aim_x))*(*(target_pub.aim_x)) + (*(target_pub.aim_y))*(*(target_pub.aim_y)));
      
      target_pub.pitch = trajectory(27.0, distance, *(target_pub.aim_z)) + pitch_diff;//正常的弹道解算
        // RCLCPP_INFO(this->get_logger(),"yaw_change");
      
      // //控制pnp因距离远近 或 v_yaw 大小 带来的aim_z误差，
      // if(distance > 3.0 && distance < 5.8){
      //   // diff_control = 0.01+0.02*((distance-3)/3); //0.02*((distance-3)/3); //在3米以外，随距离增加，解算误差增大速度也加快
      //   diff_control = 0.3 + 0.4*((distance-3)/2.8); //0.02*((distance-3)/3); //在3米以外，随距离增加，解算误差增大速度也加快
      //   if(fabsf(target_pub.get_v_yaw) > 4){
      //   RCLCPP_INFO(this->get_logger(),"yaw_change");

      //   diff_control = 0.6 + 0.4*((distance-3)/2.8); //0.02*((distance-3)/3); //在3米以外，随距离增加，解算误差增大速度也加快
      //   }
      //   target_pub.pitch = trajectory(27.0, distance, *(target_pub.aim_z))-diff_control;
      // } else if (distance > 5.8 ){
      //   // diff_control = 0.06+0.06*((distance-6)/0.5) ;//0.02*((distance-1.5)/1.5);//距离
      //   diff_control = 0.7 +0.2*((distance-5.8)/0.5);//角度
      //   // RCLCPP_INFO(this->get_logger(),"yaw_change");
      //   if(fabsf(target_pub.get_v_yaw) > 4){
      //   diff_control = 1.1 +0.2*((distance-5.8)/0.5);//角度
      //   }
      //   target_pub.pitch = trajectory(27.0, distance, *(target_pub.aim_z)) - diff_control;
      // }
      // else{
	    //   diff_control =  0.02;//0.03*(distance/1.5);     
	    //   // diff_control =  0.2;//0.03*(distance/1.5);      
      //   target_pub.pitch = trajectory(29.5, distance, *(target_pub.aim_z))-diff_control;
      // }


      //弧度转角度  
      // float yaw_change = time_delay_set * 0.5;
      // RCLCPP_INFO(this->get_logger(),"yaw_change:%f",yaw_change);

      if(fabsf(target_pub.get_v_yaw)> 5.3 || (distance > 4.2 && fabsf(target_pub.get_v_yaw)> 2.5) ){
        target_pub.yaw = (float)(atan2(target_pub.get_position_y, target_pub.get_position_x))*57.2957 + yaw_diff ;

      } else{

      //对于较低转速的目标应当采取转到位置就击打的策略
      target_pub.yaw = (float)(atan2(*(target_pub.aim_y), *(target_pub.aim_x)))*57.29577 + yaw_diff ; 
      if(fabsf(*target_pub.fire - robo_yaw) < 0.7){
        *(target_pub.fire)= robo_yaw;         
      }
      }

    //
    // if(armors_num == 2 && (!armors_msg->armors.empty())){
    // target_pub.yaw = (float)(atan2(*(target_pub.aim_y), *(target_pub.aim_x)))*57.29577 + yaw_diff ; 
    // *(target_pub.fire)= robo_yaw;    
    // }

    //保护目标角度 在 -180 到 +180 区间
      if(target_pub.yaw > 180){
            target_pub.yaw -=360;
      }else if (target_pub.yaw < -180.0)  {
            target_pub.yaw +=360;
      }

    //计算相对于当前位置，自身在yaw上要转动的角度
      float mv_yaw =target_pub.yaw - robo_yaw*57.2957;
      if(mv_yaw > 180){
        mv_yaw -= 360;
      } else if (mv_yaw < -180.0){
        mv_yaw += 360;
      }
        // RCLCPP_INFO(this->get_logger(),"mv_yaw:%f",mv_yaw);
      //根据移动角度判断是否开火，以及检测所发位置是否正常
      if(fabsf(mv_yaw)>5){
        RCLCPP_WARN(this->get_logger(),"this is a peculiar mv_yaw:%f",mv_yaw);
        *(target_pub.fire) = 10;
      } else if(fabsf(mv_yaw)> 20){
        target_pub.yaw = last_pre_yaw;
      }
      //由于目前建系未考虑自身和目标体积 在距离过近时 会导致旋转角度过大的情况出现，避免此类情况出现
      if(distance < 0.4)
      {
        std::cout<<"enemy lost "<<std::endl;
        tracker_->tracker_state = Tracker::LOST;
      }

      last_pre_yaw = target_pub.yaw;
      target_msg.pre_pitch = target_pub.pitch;
      target_msg.pre_yaw = target_pub.yaw;
      target_msg.aim_x = *(target_pub.aim_x);
      target_msg.aim_y = *(target_pub.aim_y);
      target_msg.aim_z = *(target_pub.aim_z);
      target_msg.fire = *(target_pub.fire);
      // test_msg 用于查看你想监控的数据
      test_msg.data = *(target_pub.fire);
    }
  }

  last_time_ = time;
  test_pub_->publish(test_msg);                                     
  target_pub_->publish(target_msg);

  publishMarkers(target_msg);
}

void ArmorTrackerNode::publishMarkers(const auto_aim_interfaces::msg::Target & target_msg)
{
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armor_marker_.header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;
  if (target_msg.tracking) {
    double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y, za = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
    double dz = target_msg.dz;

    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = za + dz / 2;

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;
    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      // Only 4 armors has 2 radius and height
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        p_a.z = za + (is_current_pair ? 0 : dz);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        p_a.z = za;
      }
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);

      armor_marker_.id = i;
      armor_marker_.pose.position = p_a;
      tf2::Quaternion q;
      q.setRPY(0, target_msg.id == "outpost" ? -0.26 : 0.26, tmp_yaw);
      armor_marker_.pose.orientation = tf2::toMsg(q);
      marker_array.markers.emplace_back(armor_marker_);
    }
  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETEALL;

    armor_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.emplace_back(armor_marker_);
  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_pub_->publish(marker_array);
}

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::ArmorTrackerNode)
