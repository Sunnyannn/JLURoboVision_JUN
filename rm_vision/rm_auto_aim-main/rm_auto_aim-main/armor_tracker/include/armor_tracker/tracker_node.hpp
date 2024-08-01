// Copyright 2022 Chen Jun

#ifndef ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
#define ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_

// ROS
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>


#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "armor_tracker/tracker.hpp"
#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/tracker_info.hpp"

namespace rm_auto_aim
{
using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Armors>;
class ArmorTrackerNode : public rclcpp::Node
{
public:
  explicit ArmorTrackerNode(const rclcpp::NodeOptions & options);

private:
  void armorsCallback(const auto_aim_interfaces::msg::Armors::SharedPtr armors_ptr);

  void yawCallback(const std_msgs::msg::Float64 msg);

  void sentryCallback(const std_msgs::msg::Int8 msg);

  void publishMarkers(const auto_aim_interfaces::msg::Target & target_msg);

  void limit_yaw_range_360(float& pre_yaw); 

  // Maximum allowable armor distance in the XOY plane
  double max_armor_distance_;

  // The time when the last message was received
  rclcpp::Time last_time_;
  double dt_;

  // Armor tracker
  double last_pre_yaw;
  double pitch_diff;
  double yaw_diff;

  double s2qxyz_max_, s2qxyz_min_,s2qyaw_max_,s2qyaw_min_, s2qr_;
  double r_xyz_factor, r_yaw;
  double lost_time_thres_;
  std::unique_ptr<Tracker> tracker_;

  // Reset tracker service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tracker_srv_;

  ///////
  int respond_time = 0; //用于设置响应时间
  int highest_level = 3; //从3开始检索当前所有装甲板的优先级，从0开始,优先级依次下降
  int priority_level = 0; //当前优先级
  int sentry_decision = 0;//用于判断对敌方哨兵击打策略
  int armors_num = 4;   //用于击打目标建模
  float robo_yaw=0;   //储存下位机回传的当前yaw值
  float diff_control = 0.0; // 控制因为距离带来的pnp垂直误差
  float yaw_control = 0.0; //用于人工控制yaw
  float v_judge = 0.0; //用于判断敌方是否为小陀螺，更改开火逻辑
  float v_yaw_judge = 0.0; //同上
  float fire_judge = 0.0; //用于判断 是否对低转速进行开火
  
 
  // Subscriber with tf2 message_filter
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<auto_aim_interfaces::msg::Armors> armors_sub_;
  std::shared_ptr<tf2_filter> tf2_filter_;

  // Tracker info publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::TrackerInfo>::SharedPtr info_pub_;

  // Publisher
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;

  // Visualization marker publisher
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker linear_v_marker_;
  visualization_msgs::msg::Marker angular_v_marker_;
  visualization_msgs::msg::Marker armor_marker_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  //struct
  struct pub_str
  {
      float pitch;
      float yaw;
      float *aim_x = new float;
      float *aim_y = new float;
      float *aim_z = new float;
      //数据提取
      float get_position_x= 0;
      float get_position_y= 0;
      float get_position_z= 0;
      float get_velocity_x= 0;
      float get_velocity_y= 0;
      float get_velocity_z= 0;
      float get_v_yaw = 0;
      float get_r1 = 0;
      float *fire = new float;
  };

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sentry_decision_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr test_pub_;
  std_msgs::msg::Float64 test_msg;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__PROCESSOR_NODE_HPP_
