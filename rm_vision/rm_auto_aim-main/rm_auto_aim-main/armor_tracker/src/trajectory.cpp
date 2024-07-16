#include "armor_tracker/trajectory.h"

double PredictPitchXY::v0now;

PredictPitchXY PredictPitchXY::instance;  // 一定要初始化静态变量

double PredictPitchXY::setBulletSpeed(float bulletSpeed)
{
  if (bulletSpeed != 0)
  {
    return bulletSpeed;
  }
  else
  {
    return 15.7;
  }
}

void PredictPitchXY::InitPredictPitch(float bulletSpeedNow, double distance, float hrRec)
{
  // 赋值初始量
  v0now = setBulletSpeed(bulletSpeedNow);
  X_r = distance;
  Y_r = hrRec;
}

// 8.27：应当以时间为单位作为步长，或者说我下面使用的步长正是时间而我理解为了距离（受到沈航代码先入为主的影响），所以才会出现角度变动如此之大和稳定的问题
double PredictPitchXY::dropshotRK45()
{
  double error = 0;
  theta = 5 * 3.14 / 180.0;  // 初始化角度
  for (int i = 0; i < maxstep; i++)
  {  // 设置最大步长
    // 初始化

    time_acc = 0;
    theta_d = theta;  // 将theta赋给过程量
    X_d = 0.20* cos(theta_d);   // 2024infantry枪管相对于云台中心的水平长度
    Y_d = 0.20* sin(theta_d) ;  // 2024infantry枪管相对于云台中心的垂直高度
    v0_d = v0now;

    double time_step = 0.0004;  // 时间步长

    while (X_d < X_r)  // 迭代
    {
      // 计算各阶

      auto k1_v = (-k * pow(v0_d, 2) - g * sin(theta_d)) * time_step;
      auto k1_theta = (-g * cos(theta_d) / v0_d) *
                      time_step;  // 由于大弹丸很难产生马格努斯效应（横向摩擦轮怎么可能产生后向旋转），故忽略升力
      // 其实这里和公式是不完全相同的，未考虑t随步长的改变，因为t在推导过程中简化约去了

      auto k1_v_2 = v0_d + k1_v / 4.0;
      auto k1_theta_2 = theta_d + k1_theta / 4.0;

      auto k2_v = (-k * pow(k1_v_2, 2) - g * sin(k1_theta_2)) * time_step;
      auto k2_theta = (-g * cos(k1_theta_2) / k1_v_2) * time_step;
      auto k12_v_3 = v0_d + 3.0 / 32.0 * k1_v + 9.0 / 32.0 * k2_v;
      auto k12_theta_3 = theta_d + 3.0 / 32.0 * k1_theta + 9.0 / 32.0 * k2_theta;

      auto k3_v = (-k * pow(k12_v_3, 2) - g * sin(k12_theta_3)) * time_step;
      auto k3_theta = (-g * cos(k12_theta_3) / k12_v_3) * time_step;
      auto k123_v_4 = v0_d + 1932.0 / 2179.0 * k1_v - 7200.0 / 2179.0 * k2_v + 7296.0 / 2179.0 * k3_v;
      auto k123_theta_4 =
          theta_d + 1932.0 / 2179.0 * k1_theta - 7200.0 / 2179.0 * k2_theta + 7296.0 / 2179.0 * k3_theta;

      auto k4_v = (-k * pow(k123_v_4, 2) - g * sin(k123_theta_4)) * time_step;
      auto k4_theta = (-g * cos(k123_theta_4) / k123_v_4) * time_step;
      auto k1234_v_5 = v0_d + 439.0 / 216.0 * k1_v - 8.0 * k2_v + 3680.0 / 513.0 * k3_v - 845.0 / 4140.0 * k4_v;
      auto k1234_theta_5 =
          theta_d + 439.0 / 216.0 * k1_theta - 8.0 * k2_theta + 3680.0 / 513.0 * k3_theta - 845.0 / 4140.0 * k4_theta;

      auto k5_v = (-k * pow(k1234_v_5, 2) - g * sin(k1234_theta_5)) * time_step;
      auto k5_theta = (-g * cos(k1234_theta_5) / k1234_v_5) * time_step;
      auto k12345_v_6 =
          v0_d - 8.0 / 27.0 * k1_v + 2.0 * k2_v - 3544.0 / 2565.0 * k3_v + 1859.0 / 4104.0 * k4_v - 11.0 / 40.0 * k5_v;
      auto k12345_theta_6 = theta_d - 8.0 / 27.0 * k1_theta + 2.0 * k2_theta - 3544.0 / 2565.0 * k3_theta +
                            1859.0 / 4104.0 * k4_theta - 11.0 / 40.0 * k5_theta;

      auto k6_v = (-k * pow(k12345_v_6, 2) - g * sin(k12345_theta_6)) * time_step;
      auto k6_theta = (-g * cos(k12345_theta_6) / k12345_v_6) * time_step;

      // 计算近似解以及相对误差
      // 注：计算相对误差采用五阶精度与四阶精度系数结果之差
      // auto vclass_4 = v0_d + 25.0 / 216.0 * k1_v + 1408.0 / 2565.0 * k3_v + 2197.0 / 4104.0 * k4_v - 1.0 / 5.0 *
      // k5_v; auto thetaclass_4 = theta_d + 25.0 / 216.0 * k1_theta + 1408.0 / 2565.0 * k3_theta + 2197.0 / 4104.0 *
      // k4_theta - 1.0 / 5.0 * k5_theta;

      auto vclass_5 = v0_d + 16.0 / 135.0 * k1_v + 6656.0 / 12825.0 * k3_v + 28561.0 / 56430.0 * k4_v -
                      9.0 / 50.0 * k5_v + 2.0 / 55.0 * k6_v;
      auto thetaclass_5 = theta_d + 16.0 / 135.0 * k1_theta + 6656.0 / 12825.0 * k3_theta +
                          28561.0 / 56430.0 * k4_theta - 9.0 / 50.0 * k5_theta + 2.0 / 55.0 * k6_theta;

      // auto error_v = abs(vclass_5 - vclass_4);
      // auto error_theta = abs(thetaclass_5 - thetaclass_4);
      // auto error_y = tan(error_theta) * time_step;

      // std::cout << "error_v:" << error_v << std::endl;
      // std::cout << "error_theta:" << error_theta << std::endl;
      // std::cout << "error_y:" << error_y << std::endl;

      v0_d = vclass_5;
      theta_d = thetaclass_5;
      X_d += time_step * v0_d * cos(theta_d);
      Y_d += time_step * v0_d * sin(theta_d);
      time_acc += time_step;
      if (abs(theta_d) > limit_pitch)
      {
        if (over_limit_judge != 0 && fabsf(theta_d) > over_limit_judge)
        {
          theta_d = over_limit_judge;
          std::cout << " trajectory is over limit" << std::endl;
          break;
        }
        over_limit_judge = abs(theta_d);
      }
    }
    // 评估迭代结果，修正theta
    error = Y_r - Y_d;  // error可以pid调节
    if (abs(error) < minerr_y)
    {
      // std::cout << i << std::endl;
      // std::cout << "..............................................." << std::endl;
      // std::cout << "theta:" << theta * 180 / 3.1415 << std::endl;
      // std::cout << "..............................................." << std::endl;
      // std::cout <<"error:"<< error << std::endl;
      return theta;
    }  // 合适则输出本次迭代使用的theta
    else
    {
      theta += atan((error) / X_r);
      // std::cout << "...................................................." << std::endl;
      // std::cout << "error:" << error << std::endl;
      // std::cout << "theta_d:" << theta_d * 180 / 3.1415 << endl;
      // std::cout << "...................................................." << std::endl;
    }
  }

  // 迭代失败则输出上次的迭代值(暂定为0)
  return 0;
}


void PredictPitchXY::zero_cross_detector(float& yaw_diff){
  if(yaw_diff > 5.7){
    yaw_diff = 2*pi - yaw_diff;
  }
}      

void PredictPitchXY::limit_yaw_range(float& pre_yaw){
  if(pre_yaw > pi){
        pre_yaw -=2 *pi;
      }
    if(pre_yaw < -pi){
        pre_yaw +=2 *pi;
      }
}

//一些参数

void PredictPitchXY::GimbalControlTransform(float xw, float yw, float zw, float vxw, float vyw, float vzw, float v_yaw,
                                            float r1, float r2, float dz, int armors_num, float yaw, float* aim_x, float* aim_y,
                                            float* aim_z,float *fire,float robo_yaw,float v)
{
  //距离偏置修正，一般不修，主要是角度问题
  float static_x = 0.00;
  float static_z = 0.00;
  // 线性预测
  float algorithm_time = 25; //可以通过latency查看
  float respond_time = 60;   //可以通过打小陀螺测试得到
  if(fabsf(v_yaw) > 5){
      //对于高速旋转的目标，采取秒准中心的策略，respond_time 理应更少
      respond_time = 27;
  }
  // std::cout<<"respond_time:"<<respond_time<<std::endl;
  float bias_time = algorithm_time + respond_time;  // bias_time 作为1、上位机 图像传输、算法解算；2、通信 传输耗时；3、下位机 拨弹响应、子弹加速 的趋于固定的时间损耗
  float timeDelay =  bias_time/1000 + time_acc;//  time_acc--子弹飞行时间 
  //当前相机观测到的装甲板相对于 odom坐标系 的坐标
  float tar_yaw = yaw;
  // 选择的装甲板
  int idx = 0;  

  //以下 存在一个 大的 if else 判断 当检测到目标转速较低时瞄准摄像头看见的那块装甲板，高转速再建模预测
  if(fabsf(v_yaw) < 3.0){
    if(v > 0.08){ 
      // std::cout<<"i am in here" << std::endl;
      if(fabsf(v_yaw) < 1.8){
        v_yaw = 0;
      }
      float vx = v_yaw * r1 * sin(tar_yaw);
      float vy = -v_yaw * r1 * cos(tar_yaw);
      pre_aim[0].x =xw  + vxw * timeDelay + vx * timeDelay;
      pre_aim[0].y =yw  + vyw * timeDelay + vy * timeDelay;
      pre_aim[0].z =zw  + vzw * timeDelay;
      // pre_aim[0].x =xw  + vxw * timeDelay ;
      // pre_aim[0].y =yw  + vyw * timeDelay ;
      
      pre_aim[0].yaw = tar_yaw + v_yaw * timeDelay;
    }else{
      // std::cout<<"i am in here" << std::endl;
      float vx = v_yaw * r1 * sin(tar_yaw);
      float vy = -v_yaw * r1 * cos(tar_yaw);
      pre_aim[0].x =xw - r1 * cos(tar_yaw) + vxw * timeDelay + vx * timeDelay;
      pre_aim[0].y =yw - r1 * sin(tar_yaw) + vyw * timeDelay + vy * timeDelay;
      pre_aim[0].z =zw + vzw * timeDelay;
      pre_aim[0].yaw = tar_yaw + v_yaw * timeDelay;
    }
  }else{
  
  
  //装甲板建模预测
  int i = 0;
  // 根据装甲板数目建模
  if (armors_num == 2)
  {
    for (i = 0; i < 2; i++)
    {
      float tmp_yaw = tar_yaw + i * pi;    
      float r = r1;
      tar_position[i].x = xw - r * cos(tmp_yaw);
      tar_position[i].y = yw - r * sin(tmp_yaw);
      tar_position[i].z = zw;
      tar_position[i].yaw = tar_yaw + i * 2 *pi / 2.0f;
      // if(tar_position[i].yaw > pi){
      //   tar_position[i].yaw -=2 *pi;
      // }
      limit_yaw_range(tar_position[i].yaw);
      float pre_tmp_yaw = tmp_yaw + v_yaw * timeDelay;
      pre_aim[i].x =xw + vxw * timeDelay  - r * cos(pre_tmp_yaw);
      pre_aim[i].y =yw + vyw * timeDelay  - r * sin(pre_tmp_yaw);
      pre_aim[i].z =tar_position[i].z + vzw * timeDelay;
      pre_aim[i].yaw = tar_position[i].yaw + v_yaw * timeDelay;
      limit_yaw_range(pre_aim[i].yaw);
    }
    yaw_diff_min = fabsf(robo_yaw - pre_aim[0].yaw);
    zero_cross_detector(yaw_diff_min);
    // 因为是平衡步兵 只需判断两块装甲板即可
    float temp_yaw_diff = fabsf(robo_yaw - pre_aim[1].yaw);
    zero_cross_detector(temp_yaw_diff);
    if (temp_yaw_diff < yaw_diff_min)
    {
      yaw_diff_min = temp_yaw_diff;
      idx = 1;
    }
  }
  // 0--outpost, 7--base
  else if(armors_num == 3){
      for (i = 0; i < 3; i++)
    {
      float tmp_yaw = tar_yaw + i *2* pi/3.0f;    
      float r = 0.26;
      tar_position[i].x = xw - r * cos(tmp_yaw);
      tar_position[i].y = yw - r * sin(tmp_yaw);
      tar_position[i].z = zw;
      tar_position[i].yaw = tar_yaw + i * 2 *pi / 3.0f;
      // if(tar_position[i].yaw > pi){
      //   tar_position[i].yaw -=2 *pi;
      // }
      limit_yaw_range(tar_position[i].yaw);
      float pre_tmp_yaw = tmp_yaw + v_yaw * timeDelay;
      pre_aim[i].x =xw + vxw * timeDelay  - r * cos(pre_tmp_yaw);
      pre_aim[i].y =yw + vyw * timeDelay  - r * sin(pre_tmp_yaw);
      pre_aim[i].z =tar_position[i].z + vzw * timeDelay;
      pre_aim[i].yaw = tar_position[i].yaw + v_yaw * timeDelay;
      limit_yaw_range(pre_aim[i].yaw);
    }
    //计算枪管到目标装甲板yaw最小的那个装甲板
    yaw_diff_min = fabsf(robo_yaw - pre_aim[0].yaw);
    zero_cross_detector(yaw_diff_min);
    for (i = 1; i < 3; i++)
    {
      float temp_yaw_diff = fabsf(robo_yaw- pre_aim[i].yaw);
      zero_cross_detector(temp_yaw_diff);
      if (temp_yaw_diff < yaw_diff_min)
      {
        yaw_diff_min = temp_yaw_diff;
        idx = i;
      }
    }
  }
  else    //除去以上特殊情况，其余皆为四块装甲板
  {
      // std::cout<<"i am in here" << std::endl;
    for (i = 0; i < 4; i++)
    {
      float tmp_yaw = tar_yaw + i * pi / 2.0f;
      float r = use_1 ? r1 : r2;
      tar_position[i].x = xw - r * cos(tmp_yaw);
      tar_position[i].y = yw - r * sin(tmp_yaw);
      tar_position[i].z = use_1 ? zw : dz + zw;
      tar_position[i].yaw = tar_yaw + i * pi / 2.0f;
      // if(tar_position[i].yaw > pi){
      //   tar_position[i].yaw -=2 *pi;
      // }
      limit_yaw_range(tar_position[i].yaw);
      float pre_tmp_yaw = tmp_yaw + v_yaw * timeDelay;
      pre_aim[i].x =xw + vxw * timeDelay  - r * cos(pre_tmp_yaw);
      pre_aim[i].y =yw + vyw * timeDelay  - r * sin(pre_tmp_yaw);
      pre_aim[i].z =tar_position[i].z + vzw * timeDelay;
      pre_aim[i].yaw = tar_position[i].yaw + v_yaw * timeDelay;
      limit_yaw_range(pre_aim[i].yaw);
      use_1 = !use_1;
    }

    // 2种常见决策方案：
    // 1.计算枪管到目标装甲板yaw最小的那个装甲板
    // 2.计算距离最近的装甲板

    //计算距离最近的装甲板
    //	float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
    //	int idx = 0;fire = pre_aim[idx].yaw;
    //	for (i = 1; i<4; i++)
    //	{
    //		float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
    //		if (temp_dis_diff < dis_diff_min)
    //		{
    //			dis_diff_min = temp_dis_diff;
    //			idx = i;
    //		}
    //	}
    //

    //计算枪管到目标装甲板yaw最小的那个装甲板
    yaw_diff_min = fabsf(robo_yaw - pre_aim[0].yaw);
    zero_cross_detector(yaw_diff_min);
    for (i = 1; i < 4; i++)
    {
      float temp_yaw_diff = fabsf(robo_yaw- pre_aim[i].yaw);
      zero_cross_detector(temp_yaw_diff);
      if (temp_yaw_diff < yaw_diff_min)
      {
        yaw_diff_min = temp_yaw_diff;
        idx = i;
      }
    }
  }

}


//火控策略
    *fire = 0;
    // std::cout << "armor_yaw:" << pre_aim[idx].yaw << std::endl;
    // std::cout << "robo_yaw:" << robo_yaw << std::endl;
    // std::cout << "yaw_diff_min:" << fabsf(pre_aim[idx].yaw - robo_yaw) << std::endl;
    // float r = use_1 ? r2 : r1;
    // float test = (float)(atan2(0.03,r));
    // std::cout << "r:" << r << std::endl;
    // std::cout << "test:" << tar_position[0].x << std::endl;
    
    //开火逻辑判断 v_yaw<0.5作为一个静止误差，转动速率小于 0.5rad/s 视为静止,直接击打
    //对于v_yaw在1.8rad/s以下的目标，主要是平移 加 一些慢速旋转，全力开火
    if(fabsf(v_yaw)<1.8 )
    {
      *fire = robo_yaw;
      count++;
      std::cout << "num:" << count<< std::endl;
      if(count > 10000){
        count = 0;
      }
    }
    else
    {
      // *fire = 10; 
      *fire = pre_aim[idx].yaw; //把预测的目标装甲板相对于 odom坐标系的朝向角 发给下位机用于火控判断
      // std::cout<<"it the armor --"<< idx<<std::endl;
      // std::cout<<"it the armor2 --"<< pre_aim[0].yaw<<std::endl;
      // std::cout<<"it the armor3 --"<< pre_aim[1].yaw<<std::endl;
    }
    if(yaw_diff_min < 0.06 && fabsf(v_yaw) > 1.8){
   //把预测的目标装甲板相对于 odom坐标系的朝向角 发给下位机用于火控判断
        count++;
      if(count > 10000){
        count = 0;
      }
      std::cout << "num:" << count<< std::endl;
    }
    if(fabsf(v_yaw)>5){
      static_z += 0.00;
    }
  //把 以为 原点的坐标系 修正到云台中心
  *aim_x = pre_aim[idx].x - static_x;
  *aim_y = pre_aim[idx].y ;
  *aim_z = pre_aim[idx].z + static_z;
  // *aim_z = tar_position[idx].x - static_x;


}