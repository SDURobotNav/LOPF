#include <mutex>
#include <memory>
#include <iostream>
#include <boost/format.hpp>
#include <typeinfo>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <pcl/filters/filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <hdl_people_tracking/TrackArray.h>
#include <hdl_people_tracking/ClusterArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <people_msgs/People.h>
#include <eigen3/Eigen/Core>
#include <kkl/cvk/cvutils.hpp>
#include <hdl_people_tracking/people_tracker.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml.hpp>

#include <boost/array.hpp>

#include<thread> 
#include<mutex>
#include<stdlib.h>

using namespace cv;
using namespace cv::ml;
float backData[10][145] ={158, 9.061999986352697, 0.0, 0.0, 0.0, 2.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 7.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 7.0, 13.0, 0.0, 0.0, 0.0, 2.0, 6.0, 6.0, 10.0, 0.0, 0.0, 0.0, 4.0, 1.0, 1.0, 9.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 9.0, 1.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 6.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 9.0, 0.0, 0.0, 3.0, 2.0, 6.0, 0.0, 0.0, 8.0, 4.0, 13.0, 0.0, 1.0, 15.0, 2.0, 9.0, 0.0, 9.0, 7.0, 2.0, 8.0, 0.0, 1.0, 7.0, 1.0, 7.0, 0.0, 2.0, 8.0, 0.0, 8.0, 2.0, 0.0, 4.0, 0.0, 6.0, 4.0, 0.0, 0.0, 0.0, 7.0, 1.0, 0.0, 0.0,
36, 6.730000075611498, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 2.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 3.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 3.0, 2.0, 0.0, 1.0, 3.0, 2.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 1.0, 4.0, 1.0, 0.0, 0.0,
231, 6.488000507170559, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 14.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.0, 9.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 8.0, 13.0, 7.0, 0.0, 0.0, 0.0, 1.0, 4.0, 6.0, 6.0, 0.0, 0.0, 0.0, 2.0, 1.0, 2.0, 14.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 9.0, 8.0, 0.0, 0.0, 0.0, 1.0, 3.0, 6.0, 8.0, 7.0, 0.0, 0.0, 0.0, 0.0, 3.0, 4.0, 7.0, 7.0, 0.0, 0.0, 0.0, 2.0, 1.0, 6.0, 11.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 19.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 5.0, 9.0, 5.0, 0.0, 1.0, 7.0, 7.0, 7.0, 3.0, 5.0, 13.0, 11.0, 8.0, 5.0, 7.0, 5.0, 6.0, 4.0, 3.0, 10.0, 9.0, 7.0, 6.0, 2.0, 6.0, 5.0, 8.0, 10.0, 5.0, 4.0, 14.0, 8.0, 5.0, 6.0, 7.0, 2.0, 0.0, 0.0, 2.0, 4.0, 0.0, 0.0, 0.0, 0.0,
39, 8.009999599342818, 0.0, 6.0, 3.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 2.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0, 1.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0, 1.0, 1.0, 0.0, 5.0, 5.0, 6.0, 1.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 7.0, 6.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 7.0,
435, 5.722000193802159, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 21.0, 0.0, 0.0, 10.0, 1.0, 0.0, 8.0, 21.0, 0.0, 0.0, 0.0, 0.0, 2.0, 52.0, 0.0, 0.0, 0.0, 0.0, 0.0, 15.0, 22.0, 0.0, 0.0, 0.0, 0.0, 12.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 34.0, 0.0, 0.0, 0.0, 0.0, 0.0, 22.0, 13.0, 0.0, 0.0, 0.0, 0.0, 0.0, 28.0, 0.0, 0.0, 0.0, 0.0, 0.0, 8.0, 21.0, 0.0, 0.0, 0.0, 0.0, 0.0, 23.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 27.0, 0.0, 0.0, 0.0, 0.0, 0.0, 8.0, 18.0, 0.0, 0.0, 0.0, 0.0, 0.0, 20.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 38.0, 0.0, 0.0, 12.0, 0.0, 53.0, 0.0, 20.0, 0.0, 16.0, 36.0, 0.0, 0.0, 0.0, 0.0, 4.0, 49.0, 0.0, 0.0, 0.0, 0.0, 48.0, 0.0, 0.0, 0.0, 0.0, 45.0, 0.0, 0.0, 0.0, 0.0, 42.0, 0.0, 0.0, 0.0, 0.0, 39.0, 0.0, 0.0, 0.0, 0.0, 33.0, 0.0, 0.0, 0.0,
89, 5.414000055950855, 0.0, 2.0, 2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 6.0, 0.0, 0.0, 0.0, 1.0, 1.0, 5.0, 4.0, 4.0, 1.0, 0.0, 4.0, 2.0, 1.0, 5.0, 1.0, 0.0, 0.0, 6.0, 0.0, 2.0, 5.0, 0.0, 0.0, 2.0, 2.0, 0.0, 0.0, 6.0, 0.0, 0.0, 2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 2.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 4.0, 2.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.0, 2.0, 7.0, 6.0, 3.0, 0.0, 0.0, 5.0, 12.0, 7.0, 0.0, 0.0, 4.0, 4.0, 8.0, 2.0, 1.0, 1.0, 0.0, 4.0, 0.0, 3.0, 2.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0,
76, 4.306000153358233, 0.0, 0.0, 0.0, 4.0, 3.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 3.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 2.0, 4.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 4.0, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 3.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 1.0, 1.0, 3.0, 7.0, 2.0, 0.0, 0.0, 1.0, 3.0, 1.0, 0.0, 1.0, 1.0, 1.0, 2.0, 12.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 2.0, 2.0, 1.0, 9.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 2.0, 2.0, 4.0, 0.0, 1.0, 0.0, 0.0, 10.0, 0.0, 0.0, 3.0, 0.0, 0.0,
29, 5.4240002041950115, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 1.0, 1.0, 1.0, 1.0, 2.0, 1.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 2.0, 2.0, 0.0, 1.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 3.0, 0.0, 1.0, 0.0, 0.0,
250, 5.358000025635688, 0.0, 0.0, 0.0, 3.0, 7.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 7.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 2.0, 3.0, 0.0, 1.0, 0.0, 8.0, 8.0, 10.0, 17.0, 2.0, 19.0, 12.0, 6.0, 4.0, 0.0, 5.0, 0.0, 7.0, 0.0, 2.0, 0.0, 1.0, 0.0, 0.0, 11.0, 24.0, 6.0, 0.0, 2.0, 1.0, 0.0, 3.0, 3.0, 6.0, 0.0, 0.0, 0.0, 0.0, 6.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 3.0, 0.0, 1.0, 0.0, 1.0, 19.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 4.0, 6.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 5.0, 5.0, 5.0, 1.0, 0.0, 0.0, 0.0, 9.0, 0.0, 0.0, 0.0, 2.0, 4.0, 0.0, 0.0, 9.0, 35.0, 17.0, 13.0, 20.0, 8.0, 8.0, 15.0, 14.0, 2.0, 3.0, 14.0, 2.0, 2.0, 0.0, 2.0, 12.0, 5.0, 0.0, 0.0, 4.0, 14.0, 12.0, 0.0, 0.0, 3.0, 0.0, 5.0, 0.0, 0.0,
19, 7.987999953529203, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 2.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 4.0};

namespace hdl_people_tracking {

class HdlPeopleTrackingNodelet : public nodelet::Nodelet {
public:

  long int target_id;
  // std::thread svm_thread;
  using PointT = pcl::PointXYZI;
  HdlPeopleTrackingNodelet() {


  }
  virtual ~HdlPeopleTrackingNodelet() {}
  void onInit() override {
    nh = getNodeHandle();
    mt_nh = getMTNodeHandle();
    private_nh = getPrivateNodeHandle();

    private_nh.param<bool>("using_svm",using_svm,true);

    tracker.reset(new PeopleTracker(private_nh));
    color_palette = cvk::create_color_palette(16);
    temp_num = 0;
    target_id =1;
    target_marker.ns = "target";
    target_marker.type = 2;
    target_marker.action = 0;
    target_marker.color.r = 1;
    target_marker.color.g = 0;
    target_marker.color.b = 0;
    target_marker.color.a = 1;
    target_marker.scale.x = 1;
    target_marker.scale.y = 1;
    target_marker.scale.z = 1;
    target_marker.header.frame_id = "velodyne";
    pos_marker.ns = "pos marker";
    pos_marker.type = 2;
    pos_marker.action = 0;
    pos_marker.color.r = 0;
    pos_marker.color.g = 1;
    pos_marker.color.b = 0;
    pos_marker.color.a = 1;
    pos_marker.scale.x = 0.3;
    pos_marker.scale.y = 0.3;
    pos_marker.scale.z = 0.3;
    pos_marker.header.frame_id = "velodyne";
    neg_marker.ns = "neg marker";
    neg_marker.type = 2;
    neg_marker.action = 0;
    neg_marker.color.r = 0;
    neg_marker.color.g = 1;
    neg_marker.color.b = 1;
    neg_marker.color.a = 1;
    neg_marker.scale.x = 0.3;
    neg_marker.scale.y = 0.3;
    neg_marker.scale.z = 0.3;
    // neg_marker.D
    neg_marker.header.frame_id = "velodyne";
    neg_num = max_train_samples - max_train_samples/3;
    // 
    pos_num = 10;
    thre_num = neg_num;
    svm->setType(SVM::C_SVC);
    svm->setC(0.1);
    svm->setKernel(SVM::LINEAR);
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER,(int)1e7,1e-6));
    train_data = Mat::zeros(max_train_samples, 145, CV_32F);
    labels = Mat::zeros(max_train_samples,1, CV_32S);


    tracks_pub = private_nh.advertise<hdl_people_tracking::TrackArray>("tracks", 10);
    marker_pub = private_nh.advertise<visualization_msgs::MarkerArray>("markers", 10);
    target_pub = nh.advertise<visualization_msgs::Marker>("target",10);
    clusters_sub = nh.subscribe("/myClusters", 1, &HdlPeopleTrackingNodelet::callback, this);
    Click_sub = nh.subscribe("/clicked_point",1, &HdlPeopleTrackingNodelet::choose_target, this);
    svmFlag_pub = nh.advertise<std_msgs::Bool>("svmFlag_pub",10);
    target_po_pub = nh.advertise<geometry_msgs::PointStamped>("/mytarget_position",1);
    svmFlag_sub = nh.subscribe("svmFlag_pub",1,&HdlPeopleTrackingNodelet::svm_train,this);
    lostFlag_pub = nh.advertise<std_msgs::Bool>("lost_flag",10);
    if(using_svm)
    {
      ROS_INFO("Using SVM");
    }
    else
    {
      ROS_INFO("Not using SVM");
    }
    // Initialize the data
    for(int i = 0;i<10;i++)
    {
      float* train_dataptr = train_data.ptr<float>(i);
      short int* train_labelptr = labels.ptr<short int>(i);
      train_labelptr[0]=0;

      for(int j = 0;j<145;j++)
      {
        train_dataptr[j] = backData[i][j];

      }
    }   
  }

private:
  void choose_target(const geometry_msgs::PointStampedConstPtr& point_msg)
  {
    ROS_INFO("Receive a Target!!!!!");
    rec_point_flag = true;
    // rec_point_flag = !rec_point_flag;
    if (rec_point_flag)
    {
      target_id = 1;
      target_point.x = point_msg->point.x;
      target_point.y = point_msg->point.y;
      target_point.z = point_msg->point.z;
    }
  }

  void callback(const hdl_people_tracking::ClusterArrayPtr& clusters_msg) {
    // remove non-human detections
    // auto remove_loc = std::remove_if(clusters_msg->clusters.begin(), clusters_msg->clusters.end(), [=](const Cluster& cluster) { return !cluster.is_human; });
    // clusters_msg->clusters.erase(remove_loc, clusters_msg->clusters.end());

    // update people tracker
    tracker->predict(clusters_msg->header.stamp);
    tracker->correct(clusters_msg->header.stamp, clusters_msg->clusters);
    ROS_INFO("Size of Cluster is %d",clusters_msg->clusters.size());
    // publish tracks msg
    if(tracks_pub.getNumSubscribers()) {
      tracks_pub.publish(create_tracks_msg(clusters_msg->header));
    }

    // publish rviz markers
    if(marker_pub.getNumSubscribers()) {
      marker_pub.publish(create_tracked_people_marker(clusters_msg->header));
    }
  }

  hdl_people_tracking::TrackArrayConstPtr create_tracks_msg(const std_msgs::Header& header) const {
    hdl_people_tracking::TrackArrayPtr tracks_msg(new hdl_people_tracking::TrackArray());
    tracks_msg->header = header;

    tracks_msg->tracks.resize(tracker->people.size());
    for(int i=0; i<tracker->people.size(); i++) {
      const auto& track = tracker->people[i];
      auto& track_msg = tracks_msg->tracks[i];

      track_msg.id = track->id();
      track_msg.age = (track->age(header.stamp)).toSec();
      track_msg.pos.x = track->position().x();
      track_msg.pos.y = track->position().y();
      track_msg.pos.z = track->position().z();
      track_msg.vel.x = track->velocity().x();
      track_msg.vel.y = track->velocity().y();
      track_msg.vel.z = track->velocity().z();

      Eigen::Matrix3d pos_cov = track->positionCov();
      for(int k=0; k<3; k++) {
        for(int j=0; j<3; j++) {
          track_msg.pos_cov[k*3 + j] = pos_cov(k, j);
        }
      }

      Eigen::Matrix3d vel_cov = track->velocityCov();
      for(int k=0; k<3; k++) {
        for(int j=0; j<3; j++) {
          track_msg.vel_cov[k*3 + j] = vel_cov(k, j);
        }
      }

      const Cluster* associated = boost::any_cast<Cluster>(&track->lastAssociated());
      if(!associated) {
        continue;
      }

      track_msg.associated.resize(1);
      track_msg.associated[0] = (*associated);
    }

    return tracks_msg;
  }

  visualization_msgs::MarkerArrayConstPtr create_tracked_people_marker(const std_msgs::Header& header)
  {
    visualization_msgs::MarkerArrayPtr markers_ptr(new visualization_msgs::MarkerArray());

    visualization_msgs::MarkerArray& markers = *markers_ptr;
    markers.markers.reserve(tracker->people.size() + 1);
    markers.markers.resize(1);

    visualization_msgs::Marker& boxes = markers.markers[0];
    boxes.header = header;
    boxes.action = visualization_msgs::Marker::ADD;
    boxes.lifetime = ros::Duration(1.0);

    boxes.ns = "boxes";
    boxes.type = visualization_msgs::Marker::CUBE_LIST;
    boxes.colors.reserve(tracker->people.size());
    boxes.points.reserve(tracker->people.size());

    boxes.pose.position.z = 0.0f;
    boxes.pose.orientation.w = 1.0f;

    boxes.scale.x = 0.5;
    boxes.scale.y = 0.5;
    boxes.scale.z = 1.2;

    int neg_id = 20;
    int pos_id = 25;
    float min_dis =2.0;
    target_in_range = false;
    int target_num = -1;
    lost_flag.data = true;
    for(int i=0; i<tracker->people.size(); i++) 
    {
      const auto& person = tracker->people[i];
      const auto& color = color_palette[person->id() % color_palette.size()];
      // save the feature of all cluster
      // auto cluster = tracker->my_clusters[i];
      // auto temp_feature = cluster.feature;
      if(person->correctionCount() < 5) 
      {
        // float* train_dataptr = train_data.ptr<float>(neg_num);
        // for(int j=0; j<temp_feature.size();j++)
        // {
        //   train_dataptr[j] = float(temp_feature[j]);
        // }
        // short int* train_labelptr = labels.ptr<short int>(neg_num);
        // train_labelptr[0] = 0;
        // neg_num++;
        // if(neg_num>=max_train_samples)
        // {
        //   neg_num = thre_num;
        //   neg_filled = true;
        // }
        
        continue;
      }
      std_msgs::ColorRGBA rgba;
      rgba.r = color[2] / 255.0;
      rgba.g = color[1] / 255.0;
      rgba.b = color[0] / 255.0;
      rgba.a = 0.6f;
      markers.markers[0].colors.push_back(rgba);
      geometry_msgs::Point point;
      point.x = person->position().x();
      point.y = person->position().y();
      point.z = person->position().z();
      bool move_flag = false;
      long int temp_id;
      if(rec_point_flag)
      {
        // convertVector2Mat
        float delta_dis = sqrt(pow(point.x-target_point.x,2)+pow(point.y-target_point.y,2)+pow(point.z-target_point.z,2));
        if (delta_dis < min_dis)
        {
          target_in_range = true;
          move_flag = true;
          ROS_INFO("Target is moving!!!");
          min_dis = delta_dis;
          target_num = i;
          temp_id = person->id();
          target_point.x = person->position().x();
          target_point.y = person->position().y();
          target_point.z = person->position().z();
          // if(target_po_pub.getNumSubscribers())
          geometry_msgs::PointStamped tar_PointStamp;

          tar_PointStamp.point = target_point;
          tar_PointStamp.header.stamp = header.stamp;
          target_po_pub.publish(tar_PointStamp);
          target_marker.pose.position.x = person->position().x();
          target_marker.pose.position.y = person->position().y();
          target_marker.pose.position.z = person->position().z();
          target_marker.header.stamp = header.stamp;
          target_marker.header.frame_id = "velodyne";
          target_marker.id = 1;
        }
        target_bc.sendTransform(
        tf::StampedTransform(
        tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(target_point.x,target_point.y,target_point.z)),header.stamp,"velodyne","target"));
        target_pub.publish(target_marker);
        lost_flag.data = false;
      } 
      if (move_flag)
      {
        ROS_INFO("Lost Target!!!!!!!!!!!!!!!!!!!!!");
      }
      if (first_flag==false)
      {
        first_flag=true;
        target_id = temp_id;
      }
      markers.markers[0].points.push_back(point);

      visualization_msgs::Marker text;
      text.header = markers.markers[0].header;
      text.action = visualization_msgs::Marker::ADD;
      text.lifetime = ros::Duration(1.0);

      text.ns = (boost::format("text%d") % person->id()).str();
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.scale.z = 0.5;

      text.pose.position = point;
      text.pose.position.z += 0.7;
      text.color.r = text.color.g = text.color.b = text.color.a = 1.0;
      if(person->id()!=temp_id || rec_point_flag == false)
      {
        text.text = (boost::format("id:%d") % person->id()).str();
      }
      else
      {
        text.text = (boost::format("id:%d") % target_id).str();
      }
      // markers.markers.push_back(text);

    }
    float dis2Cluster = 3.0;
    int kk = 0;
    int target_cluster = -1;
    for(auto cluster:tracker->my_clusters)
    {
      float dis = sqrt(pow(cluster.centroid.x-target_point.x,2)+pow(cluster.centroid.y-target_point.y,2)+pow(cluster.centroid.z-target_point.z,2));
      if (dis<dis2Cluster)
      {
        dis2Cluster = dis;
        target_cluster = kk;
      }
      kk+=1;
    }
    // Set the train_data and the labels
    if(target_cluster >=0&&(training_flag==false)&&rec_point_flag)
    {
      ROS_INFO("People Size is %d",tracker->people.size());
      ROS_INFO("Cluster Size is %d",tracker->my_clusters.size());
      for(int k=0; k < tracker->my_clusters.size(); k++) 
      {
        ROS_INFO("KKKKK %d",k);
        auto cluster = tracker->my_clusters[k];
        auto temp_feature = cluster.feature;
        geometry_msgs::Point center;
        center = cluster.centroid;
        // if ID is not the target ID
        if (k!=target_cluster)
        {
          float* train_dataptr = train_data.ptr<float>(neg_num);
          if(svm_finished)
          {
            Mat tempFeature = Mat::zeros(1,145,CV_32F);
            float* tmpfea = tempFeature.ptr<float>(0); 
            for(int j=0; j<temp_feature.size();j++)
            {
              train_dataptr[j] = float(temp_feature[j]);
              tmpfea[j] = float(temp_feature[j]);
            }
            short int* train_labelptr = labels.ptr<short int>(neg_num);
            train_labelptr[0] = 1;
            neg_num++;
            if(neg_num>=max_train_samples)
            {
              neg_num = thre_num;
              neg_filled = true;
            }
            pub_makers(center,markers,0,neg_id);
            neg_id++;
            publish_predict(markers,tempFeature,tracker->people[k]);
          }
          else
          {
            for(int j=0; j<temp_feature.size();j++)
            {
              train_dataptr[j] = float(temp_feature[j]);
            }
            short int* train_labelptr = labels.ptr<short int>(neg_num);
            train_labelptr[0] = 1;
            neg_num++;
            if(neg_num>=max_train_samples)
            {
              neg_num = thre_num;
              neg_filled = true;
            }
            pub_makers(center,markers,0,neg_id);
            neg_id++;
          }
        }
        // if ID is the target ID
        else
        {
          float* train_dataptr = train_data.ptr<float>(pos_num);
          if(svm_finished)
          {            
            ROS_INFO("SVM FINISHING");
            Mat tempFeature = Mat::zeros(1,145,CV_32F);
            float* tmpfea = tempFeature.ptr<float>(0); 
            for(int j=0; j<temp_feature.size();j++)
            {
              train_dataptr[j] = float(temp_feature[j]);
              tmpfea[j] = float(temp_feature[j]);
            }
            short int* train_labelptr = labels.ptr<short int>(pos_num);
            train_labelptr[0] = 2;
            pos_num++;
            if(pos_num>=thre_num)
            {
              pos_num = 10;
              pos_filled = true;
            }
            publish_predict(markers,tempFeature,tracker->people[k]);
            pub_makers(center,markers,1,pos_id);
            pos_id++;
          }
          else
          {
            for(int j=0; j<temp_feature.size();j++)
            {
              train_dataptr[j] = float(temp_feature[j]);
            }
            short int* train_labelptr = labels.ptr<short int>(pos_num);
            train_labelptr[0] = 2;
            pos_num++;
            if(pos_num>=thre_num)
            {
              pos_num = 10;
              pos_filled = true;
            }
            pub_makers(center,markers,1,pos_id);
            pos_id++;
          }
        }
      }
    }
    if(lost_flag.data==true)
    {
      num_lost++;
      if(num_lost>10)
      {
        num_lost = 0;
        lostFlag_pub.publish(lost_flag);
        ROS_INFO("Target has lost");
      }
    }
    else
    {
      lostFlag_pub.publish(lost_flag);
    }

    marker_pub.publish(markers);
    
    if(pos_filled&&neg_filled&&using_svm)
    {
      pos_filled =false;
      neg_filled = false;
      if(curFilledNum==0)
      {
        std_msgs::Bool temp_flag;
        temp_flag.data = true;
        svmFlag_pub.publish(temp_flag);
      }
      curFilledNum++;
      if(curFilledNum>5)
      {
        curFilledNum=0;
      }


    }


    return markers_ptr;
  }

  void publish_predict(visualization_msgs::MarkerArray& markers,Mat tempFeature,const auto& person)
  {
    float prediction = svm->predict(tempFeature);
    ROS_INFO("Prediction %f",prediction);
    geometry_msgs::Point point;
    point.x = person->position().x();
    point.y = person->position().y();
    point.z = person->position().z();

    visualization_msgs::Marker svm_predict;
    svm_predict.header = markers.markers[0].header;
    svm_predict.action = visualization_msgs::Marker::ADD;
    svm_predict.lifetime = ros::Duration(1.0);
    svm_predict.id = 10;
    svm_predict.ns = (boost::format("svm_predict%d") % person->id()).str();
    svm_predict.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    svm_predict.scale.z = 0.5;
    svm_predict.pose.position = point;
    svm_predict.pose.position.z += 1.4;
    svm_predict.color.r = svm_predict.color.g = svm_predict.color.b = svm_predict.color.a = 1.0;
    svm_predict.text = (boost::format("Target:%f")%prediction).str();
    
    markers.markers.push_back(svm_predict);
  }



  void svm_train(const std_msgs::BoolConstPtr &msg)
  { 
    training_flag = true;
    ROS_INFO("PPPPPPPPPPPPPPPPP");
    if(rec_point_flag)
    {
      ROS_INFO("SVM begin Training");
      svm_finished = true;
      svm->train(train_data,ROW_SAMPLE,labels);
    }
    else
    {
      ROS_INFO("SVM Thread Intialized!!!!!!!!!");
    }
    training_flag = false;
    svmFinished = ros::Time::now().sec;

    return;
    
  }
  // void pub_marker()
  void pub_makers(geometry_msgs::Point center,visualization_msgs::MarkerArray& markers,int flag,int id)
  {
    if(flag ==1)
    {
      pos_marker.id = id;
      pos_marker.pose.position = center;
      pos_marker.pose.position.z+=2.5;
      //markers.markers.push_back(pos_marker);
    }
    else
    {
      neg_marker.id = id;
      neg_marker.pose.position = center;
      neg_marker.pose.position.z+=2.5;
      //markers.markers.push_back(neg_marker);
    }
  }
private:
  // ROS
  ros::NodeHandle nh;
  ros::NodeHandle mt_nh;
  ros::NodeHandle private_nh;

  ros::Publisher tracks_pub;
  ros::Publisher marker_pub;
  ros::Publisher target_pub;
  ros::Subscriber clusters_sub;
  ros::Subscriber Click_sub;

  ros::Publisher svmFlag_pub;
  ros::Subscriber svmFlag_sub;

  ros::Publisher lostFlag_pub;

  ros::Publisher target_po_pub;
  boost::circular_buffer<cv::Scalar> color_palette;
  tf::TransformBroadcaster target_bc;

  std::unique_ptr<PeopleTracker> tracker;
  geometry_msgs::Point target_point;
  people_msgs::People target_people;

  bool rec_point_flag = false;

  int num_lost;
  std_msgs::Bool lost_flag;
  visualization_msgs::Marker target_marker;
  visualization_msgs::Marker pos_marker;
  visualization_msgs::Marker neg_marker;
  const int max_train_samples = 90;
  bool first_flag = false;
  int temp_num; // temp_num is the temp train data input

  float time = 0.01;

  
  // SVM parameters
  Ptr<SVM> svm = SVM::create();

  bool target_in_range = false;
  bool training_flag = false;
  int pos_num; // number of Positive samples
  int neg_num; // number of Negtive samples
  int thre_num;

  bool pos_filled = false;
  bool neg_filled = false;

  Mat train_data;
  Mat labels;
  bool filled_flag = false;
  bool svm_finished = false;

  int svmFinished;
  int lastFilledNum=0;
  int curFilledNum=0;
  bool using_svm;

  
};

}
PLUGINLIB_EXPORT_CLASS(hdl_people_tracking::HdlPeopleTrackingNodelet, nodelet::Nodelet)