#include <errno.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <netinet/in.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <chrono>
#include <iomanip>
#include <iostream>

#include "moving_average.h"
#include "tf/transform_datatypes.h"

using namespace std;

#define SERV_PORT 43897
#define PI 3.1415926

struct RobotState {
  int robot_basic_state;
  int robot_gait_state;
  double rpy[3];
  double rpy_vel[3];
  double xyz_acc[3];
  double pos_world[3];
  double vel_world[3];
  double vel_body[3];
  unsigned touch_down_and_stair_trot;
  bool is_charging;
  unsigned error_state;
  int auto_charge;
  double battery_level;
  int task_state;
  bool temp1;
  bool temp2;
};
#pragma pack(4)
struct RobotStateReceived {
  int code;
  int size;
  int cons_code;
  struct RobotState data;
};

struct ImuData {
  int32_t timestamp;
  union {
    float buffer_float[9];
    uint8_t buffer_byte[3][12];
    struct {
      float angle_roll, angle_pitch, angle_yaw;
      float angular_velocity_roll, angular_velocity_pitch, angular_velocity_yaw;
      float acc_x, acc_y, acc_z;
    };
  };
};
struct ImuDataReceived {
  int code;
  int size;
  int cons_code;
  struct ImuData data;
};

struct JointState {
  double LF_Joint;
  double LF_Joint_1;
  double LF_Joint_2;
  double RF_Joint;
  double RF_Joint_1;
  double RF_Joint_2;
  double LB_Joint;
  double LB_Joint_1;
  double LB_Joint_2;
  double RB_Joint;
  double RB_Joint_1;
  double RB_Joint_2;
};
struct JointStateReceived {
  int code;
  int size;
  int cons_code;
  struct JointState data;
};

int main(int argc, char **argv) {
  int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd < 0) {
    perror("socket");
    exit(1);
  }
  struct sockaddr_in addr_serv;
  int len;
  memset(&addr_serv, 0, sizeof(struct sockaddr_in));
  addr_serv.sin_family = AF_INET;
  addr_serv.sin_port = htons(SERV_PORT);
  addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);
  len = sizeof(addr_serv);
  if (bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0) {
    perror("bind error:");
    exit(1);
  }
  int recv_num = -1;
  char recv_buf[500];
  struct sockaddr_in addr_client;

  // ROS 节点
  ros::init(argc, argv, "qnx2ros");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  int filter_size;
  private_nh.param<int>("filter_size", filter_size, 1);
  bool is_vel_world;
  private_nh.param<bool>("is_vel_world", is_vel_world, true);

  MovingAverage filter_vel_x(filter_size);
  MovingAverage filter_vel_y(filter_size);
  MovingAverage filter_vel_theta(filter_size);

  ros::Publisher leg_odom_pub = nh.advertise<nav_msgs::Odometry>("leg_odom", 1);
  ros::Publisher joint_state_pub =
      nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 1);
  //ros::Publisher imu_pub_200hz = nh.advertise<sensor_msgs::Imu>("/imu", 1);
  // std::cout<<"@@@@@@@@@@@@@@@@@@"<<endl;

  std::chrono::steady_clock::time_point start_outer =
      std::chrono::steady_clock::now();
  int64_t counter_RobotState = 0;
  int64_t counter_JointState = 0;
  int64_t counter_IMURawData = 0;
  int64_t counter_sum = 0;
  ros::Rate loop_rate(500);
  while (ros::ok()) {
    if ((recv_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0,
                             (struct sockaddr *)&addr_client,
                             (socklen_t *)&len)) < 0) {
      perror("recvfrom error:");
      exit(1);
    }
    counter_sum++;

    std::chrono::duration<double> time_counter =
        std::chrono::duration_cast<std::chrono::duration<double>>(
            std::chrono::steady_clock::now() - start_outer);
    if (time_counter.count() >= 1.0) {
      if (counter_IMURawData < 200) {
        // std::cout << "counter_RobotState: " << counter_RobotState << ". "
        //           << "counter_JointState: " << counter_JointState << ". "
        //           << "counter_IMURawData: " << counter_IMURawData << ". "
        //           << "counter_sum: " << counter_sum << " using "
        //           << time_counter.count() << " seconds." << std::endl;
      }
      counter_RobotState = 0;
      counter_JointState = 0;
      counter_IMURawData = 0;
      counter_sum = 0;
      start_outer = std::chrono::steady_clock::now();
    }
    // 发布里程计数据
    if (recv_num == sizeof(RobotStateReceived)) {
      RobotStateReceived *dr = (RobotStateReceived *)(recv_buf);
      RobotState *robot_state = &dr->data;
      // ROS_INFO_STREAM("CODE: " << dr->code);
      if (dr->code == 2305) {
        nav_msgs::Odometry leg_odom_data;
        leg_odom_data.header.frame_id = "odom";
        leg_odom_data.child_frame_id = "base_link";
        // Position
        leg_odom_data.header.stamp = ros::Time::now();
        leg_odom_data.pose.pose.orientation =
            tf::createQuaternionMsgFromYaw(robot_state->rpy[2] / 180 * PI);
        leg_odom_data.pose.pose.position.x = robot_state->pos_world[0];
        leg_odom_data.pose.pose.position.y = robot_state->pos_world[1];
        leg_odom_data.pose.pose.position.z = robot_state->pos_world[2];
        // Velocity
        double yaw = robot_state->rpy[2] / 180 * PI;
        filter_vel_x.in(robot_state->vel_world[0]);
        filter_vel_y.in(robot_state->vel_world[1]);
        filter_vel_theta.in(robot_state->rpy_vel[2]);

        if (is_vel_world) {  // vel_world to vel_base
          leg_odom_data.twist.twist.linear.x =
              +filter_vel_x.out() * cos(yaw) + filter_vel_y.out() * sin(yaw);
          leg_odom_data.twist.twist.linear.y =
              -filter_vel_x.out() * sin(yaw) + filter_vel_y.out() * cos(yaw);
        } else {  // vel_base
          leg_odom_data.twist.twist.linear.x = filter_vel_x.out();
          leg_odom_data.twist.twist.linear.y = filter_vel_y.out();
        }

        leg_odom_data.twist.twist.angular.z = filter_vel_theta.out();
        leg_odom_pub.publish(leg_odom_data);

        // IMU
        sensor_msgs::Imu imu_msg;
        imu_msg.header.frame_id = "imu";
        imu_msg.header.stamp = ros::Time::now();
        auto q = tf::createQuaternionFromRPY(robot_state->rpy[0] / 180 * PI,
                                             robot_state->rpy[1] / 180 * PI,
                                             robot_state->rpy[2] / 180 * PI);
        tf::quaternionTFToMsg(q, imu_msg.orientation);
        imu_msg.angular_velocity.x = robot_state->rpy_vel[0];
        imu_msg.angular_velocity.y = robot_state->rpy_vel[1];
        imu_msg.angular_velocity.z = robot_state->rpy_vel[2];
        imu_msg.linear_acceleration.x = robot_state->xyz_acc[0];
        imu_msg.linear_acceleration.y = robot_state->xyz_acc[1];
        imu_msg.linear_acceleration.z = robot_state->xyz_acc[2];

        imu_pub.publish(imu_msg);
        counter_RobotState++;
      }
    } else if (recv_num == sizeof(JointStateReceived)) {
      JointStateReceived *dr = (JointStateReceived *)(recv_buf);
      JointState *joint_state = &dr->data;
      // ROS_INFO_STREAM("CODE: " << dr->code);
      if (dr->code == 2306) {
        sensor_msgs::JointState joint_state_data;
        joint_state_data.header.stamp = ros::Time::now();
        joint_state_data.name.resize(12);
        joint_state_data.position.resize(12);

        joint_state_data.name[0] = "LF_Joint";
        joint_state_data.position[0] = -joint_state->LF_Joint;
        joint_state_data.name[1] = "LF_Joint_1";
        joint_state_data.position[1] = -joint_state->LF_Joint_1;
        joint_state_data.name[2] = "LF_Joint_2";
        joint_state_data.position[2] = -joint_state->LF_Joint_2;

        joint_state_data.name[3] = "RF_Joint";
        joint_state_data.position[3] = -joint_state->RF_Joint;
        joint_state_data.name[4] = "RF_Joint_1";
        joint_state_data.position[4] = -joint_state->RF_Joint_1;
        joint_state_data.name[5] = "RF_Joint_2";
        joint_state_data.position[5] = -joint_state->RF_Joint_2;

        joint_state_data.name[6] = "LB_Joint";
        joint_state_data.position[6] = -joint_state->LB_Joint;
        joint_state_data.name[7] = "LB_Joint_1";
        joint_state_data.position[7] = -joint_state->LB_Joint_1;
        joint_state_data.name[8] = "LB_Joint_2";
        joint_state_data.position[8] = -joint_state->LB_Joint_2;

        joint_state_data.name[9] = "RB_Joint";
        joint_state_data.position[9] = -joint_state->RB_Joint;
        joint_state_data.name[10] = "RB_Joint_1";
        joint_state_data.position[10] = -joint_state->RB_Joint_1;
        joint_state_data.name[11] = "RB_Joint_2";
        joint_state_data.position[11] = -joint_state->RB_Joint_2;
        joint_state_pub.publish(joint_state_data);
        counter_JointState++;
      }
    } 
    //   else if (recv_num == sizeof(ImuDataReceived)) {
    //   ImuDataReceived *dr = (ImuDataReceived *)(recv_buf);
    //   ImuData *imu_data = &dr->data;
    //   // ROS_INFO_STREAM("CODE: " << dr->code);
    //   if (dr->code == 67841) {
    //     sensor_msgs::Imu imu_msg;
    //     imu_msg.header.frame_id = "imu";
    //     imu_msg.header.stamp = ros::Time::now();
    //     auto q = tf::createQuaternionFromRPY(imu_data->angle_roll / 180 * PI,
    //                                          imu_data->angle_pitch / 180 * PI,
    //                                          imu_data->angle_yaw / 180 * PI);
    //     tf::quaternionTFToMsg(q, imu_msg.orientation);
    //     imu_msg.angular_velocity.x = imu_data->angular_velocity_roll;
    //     imu_msg.angular_velocity.y = imu_data->angular_velocity_pitch;
    //     imu_msg.angular_velocity.z = imu_data->angular_velocity_yaw;
    //     imu_msg.linear_acceleration.x = imu_data->acc_x;
    //     imu_msg.linear_acceleration.y = imu_data->acc_y;
    //     imu_msg.linear_acceleration.z = imu_data->acc_z;
    //     imu_pub_200hz.publish(imu_msg);
    //     counter_IMURawData++;
    //   }
    // } else {
    //   ROS_INFO_STREAM("UNKNOWN recv_num: " << recv_num);
    // }

    loop_rate.sleep();
  }

  close(sock_fd);
  return 0;
}