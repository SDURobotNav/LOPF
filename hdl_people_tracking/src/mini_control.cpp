#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <stdlib.h>

#include <ctime>

#include "mini_control/input.h"

using namespace std;

boost::shared_ptr<InputSocket> input_;

// 设定为 4 字节对齐
#pragma pack(4)
struct DataSend {
  int32_t code;
  int32_t size;
  int32_t cons_code;
  double cmd_data;
};

DataSend data;

DataSend data2;

void vel_callback(geometry_msgs::TwistConstPtr msg) {
  data.code = 320;
  data.size = 8;
  data.cons_code = 1;
  data.cmd_data = msg->linear.x;
  input_->sendPacket((uint8_t*)&data, sizeof(data));

  data.code =325;
  data.size = 8;
  data.cons_code = 1;
  data.cmd_data = msg->linear.y;
  input_->sendPacket((uint8_t*)&data, sizeof(data));

  data.code = 321;
  data.size = 8;
  data.cons_code = 1;
  data.cmd_data = msg->angular.z;
  input_->sendPacket((uint8_t*)&data, sizeof(data));
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "ros2qnx");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Subscriber vel_sub = nh.subscribe("cmd_vel", 1, vel_callback);


  input_.reset(new InputSocket(private_nh));

  ros::spin();

  return 0;
}
