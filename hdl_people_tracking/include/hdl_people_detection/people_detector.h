#ifndef PEOPLE_DETECTOR_H
#define PEOPLE_DETECTOR_H

#include <memory>
#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <hdl_people_detection/cluster.hpp>
#include <hdl_people_detection/kidono_human_classifier.h>
#include <hdl_people_tracking/myClusters.h>
#include <std_msgs/Bool.h>

namespace hdl_people_detection {

/**
 * @brief People detector based on Mercel's cluster detection and Kidono's person classifier
 * @see Marcel Haselich et al., "Confidence-based pedestrian tracking in unstructured environments using 3D laser distance measurements"
 * @see https://userpages.uni-koblenz.de/~agas/Documents/Haeselich2014CBP.pdf
 * @see Kiyosumi Kidono et al., "Pedestrian Recognition Using High-definition LIDAR"
 * @see http://www.aisl.cs.tut.ac.jp/~jun/pdffiles/kidono-iv2011.pdf
 */


class PeopleDetector {
public:
  PeopleDetector(ros::NodeHandle& nh);
  ~PeopleDetector();
  // ros::NodeHandle mynh;
  ros::NodeHandle nh_cxy;
  ros::Publisher pub_clusters;
  // ros::Subscriber sub_flag;
  

  std::vector<Cluster::Ptr> detect(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud) const;
  std::vector<Cluster::Ptr> detect(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud, std::string frameID, ros::Time& timeStamp) const;
  // void handle_flag(const std_msgs::BoolConstPtr& msg);


private:
  int min_pts;
  int max_pts;
  Eigen::Array3f min_size;
  Eigen::Array3f max_size;
  
  std::unique_ptr<KidonoHumanClassifier> classifier;
};

}

#endif // PEOPLE_DETECTOR_H
