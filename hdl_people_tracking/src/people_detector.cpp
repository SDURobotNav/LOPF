#include <hdl_people_detection/people_detector.h>
#include <fstream>
#include <ostream>
#include <ros/ros.h>
#include <string>
#include <ros/package.h>
#include <hdl_people_detection/cluster_detector.hpp>
#include <hdl_people_detection/marcel_people_detector.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <hdl_people_tracking/myClusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Bool.h>

namespace hdl_people_detection {
PeopleDetector::PeopleDetector(ros::NodeHandle &nh) {
  
  pub_clusters = nh_cxy.advertise<hdl_people_tracking::myClusters>("pubClusters",1000);

  // sub_flag = nh_cxy.subscribe("/record_time",1,&PeopleDetector::handle_flag,this);
  
  min_pts = nh.param<int>("cluster_min_pts", 10);
  max_pts = nh.param<int>("cluster_max_pts", 8192);
  min_size.x() = nh.param<double>("cluster_min_size_x", 0.2);
  min_size.y() = nh.param<double>("cluster_min_size_y", 0.2);
  min_size.z() = nh.param<double>("cluster_min_size_z", 0.3);
  max_size.x() = nh.param<double>("cluster_max_size_x", 1.0);
  max_size.y() = nh.param<double>("cluster_max_size_y", 1.0);
  max_size.z() = nh.param<double>("cluster_max_size_z", 2.0);
  
  if(nh.param<bool>("enable_classification", true)) 
   {
    std::string package_path = ros::package::getPath("hdl_people_tracking");
    classifier.reset(new KidonoHumanClassifier(package_path + "/data/boost_kidono.model", package_path + "/data/boost_kidono.scale"));
  }
}

PeopleDetector::~PeopleDetector() {

}

std::vector<Cluster::Ptr> PeopleDetector::detect(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud) const 
{

  MarcelPeopleDetector marcel(min_pts, max_pts, min_size, max_size);
  auto clusters = marcel.detect(cloud);
  for(auto& cluster : clusters) {
    cluster->is_human = !classifier || classifier->predict(cluster->cloud);
  }

  return clusters;
}
std::vector<Cluster::Ptr> PeopleDetector::detect(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud, std::string frameID, ros::Time& timeStamp) const 
{
  // ROS_INFO("Detect cloud!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  extern std::vector<uint32_t> time_vec;
  
  MarcelPeopleDetector marcel(min_pts, max_pts, min_size, max_size);
  auto clusters = marcel.detect(cloud);
  auto detect_clusters = marcel.detect_clusters(cloud);
  hdl_people_tracking::myClusters my_clusters;
  my_clusters.frame_id = frameID;
  my_clusters.stamp = timeStamp;
  my_clusters.Header.frame_id = frameID;
  my_clusters.Header.stamp = timeStamp;
  for(auto& cluster :detect_clusters){
    sensor_msgs::PointCloud2 temp_cloud;
    // ROS_INFO(cluster->size());
    pcl::toROSMsg(*cluster,temp_cloud);
    temp_cloud.header.frame_id = "velodyne";
    my_clusters.clusters.push_back(temp_cloud);
    // my_clusters.clusters.push_back(&cluster);
  }
  pub_clusters.publish(my_clusters);
  // ROS_INFO("Publish Clusters");
  
  for(auto& cluster : clusters) {
    uint32_t begin_time = ros::Time::now().nsec;
    cluster->is_human = !classifier || classifier->predict(cluster->cloud);
    uint32_t end_time = ros::Time::now().nsec;
    uint32_t delta_time =end_time-begin_time;
    // ROS_INFO("Using Time %d",delta_time);
  }
  return clusters;
}



}

// PLUGINLIB_EXPORT_CLASS(hdl_people_detection::HdlPeopleDetectionNodelet,nodelet::Nodelet);
