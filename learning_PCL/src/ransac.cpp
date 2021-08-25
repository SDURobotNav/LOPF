


#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#include <opencv/cv.h>
#include<pcl/pcl_base.h>
#include <pcl/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl-1.8/pcl/sample_consensus/sac.h>
#include <pcl-1.8/pcl/sample_consensus/sac_model.h>
#include <pcl-1.8/pcl/sample_consensus/ransac.h>
#include <pcl-1.8/pcl/registration/icp.h>
#endif

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

// #include <pcl/impl/instantiate.hpp>
// #include <pcl/point_types.h>


struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)
using namespace std;
ros::Publisher ransac_pub;

typedef PointXYZIRT PointCloudIn;
class ransac
{
    public:

    ros::NodeHandle nh_;
    ros::Publisher ransac_pub;
    ros::Subscriber pointcloud_input;
    float ransac_thre;
    ransac()
    {
        pointcloud_input  =nh_.subscribe("/velodyne_points",10,&ransac::laserCloudHandler, this);
        ransac_pub = nh_.advertise<sensor_msgs::PointCloud2>("ransac",1);
        nh_.param<float>("ransac_thre",ransac_thre, 0.2);
    }
    void detectObjectsOnCloud(pcl::PointCloud<PointXYZIRT>::Ptr &cloud, pcl::PointCloud<PointXYZIRT>::Ptr &cloud_filtered)
    {      
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<PointXYZIRT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        // you can modify the parameter below
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.2);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            cout<<"error! Could not found any inliers!"<<endl;
        }
        // extract ground
        pcl::ExtractIndices<PointXYZIRT> extractor;
        extractor.setInputCloud(cloud);
        extractor.setIndices(inliers);
        extractor.setNegative(true);
        extractor.filter(*cloud_filtered);
        // vise-versa, remove the ground not just extract the ground
        // just setNegative to be true
        cout << "filter done."<<endl;

    }
    void detectObjectsOnCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_filtered)
    {        
        if (cloud->size() > 0)
        {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        // you can modify the parameter below
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(ransac_thre);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            cout<<"error! Could not found any inliers!"<<endl;
        }
        // extract ground
        pcl::ExtractIndices<pcl::PointXYZI> extractor;
        extractor.setInputCloud(cloud);
        extractor.setIndices(inliers);
        extractor.setNegative(true);
        extractor.filter(*cloud_filtered);
        // vise-versa, remove the ground not just extract the ground
        // just setNegative to be true
        cout << "filter done."<<endl;
        }
        else
        {
            std::cout<<"no data!"<<std::endl;
        }
        

    }
    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        // pcl::PointCloud<PointXYZIRT>::Ptr cloudin(new pcl::PointCloud<PointXYZIRT>());
        // pcl::PointCloud<PointXYZIRT>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZIRT>());
        pcl::PointCloud<PointCloudIn>::Ptr cloudin(new pcl::PointCloud<PointCloudIn>());
        pcl::PointCloud<PointCloudIn>::Ptr cloud_filtered(new pcl::PointCloud<PointCloudIn>());
        pcl::fromROSMsg(*msg, *cloudin);
        // pcl::fromROSMsg(*msg, *cloudin);

        detectObjectsOnCloud(cloudin,cloud_filtered);

        sensor_msgs::PointCloud2 cloud_filteredMsg;
        pcl::toROSMsg(*cloud_filtered, cloud_filteredMsg);
        

        cloud_filteredMsg.header.stamp = msg -> header.stamp;
        
        cloud_filteredMsg.header.seq = msg -> header.seq;
        cloud_filteredMsg.header.frame_id = "/velodyne";
        for(int i = 1;i< (int)cloud_filteredMsg.fields.size();i++)
            std::cout<<cloud_filteredMsg.fields[i].name;
        ransac_pub.publish(cloud_filteredMsg);
        
    }
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ransac_test");
    ransac ran;
    ros::spin();
}