#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PointStamped.h>
#include<string>
#include<std_msgs/Bool.h>
using namespace std;
float pi = 3.14159;
class pos2vel
{
private:
    /* data */

public:
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::Subscriber pos_sub;
    ros::Subscriber lost_sub;
    string veltopic;
    string postopic;
    string lost_flag_topic;
    float max_vel;
    float min_dis;
    float max_lost_time;
    float kp;
    float kp_theta;
    bool lost_flag = false;
    float max_theta;
    void pos_cb(const geometry_msgs::PointStampedConstPtr &msg);
    void lost_cb(const std_msgs::BoolConstPtr &msg);
    pos2vel(/* args */);
    ~pos2vel();
};
void pos2vel::pos_cb (const geometry_msgs::PointStampedConstPtr &msg)
{
    if(lost_flag==false)
    {
        float x = msg->point.x;
        float y = msg->point.y;
        float theta;
        float linear;
        float distance;
        distance = sqrt(pow(x,2)+pow(y,2));
        theta = kp_theta*atan2(y, x);
        linear = kp*distance;
        geometry_msgs::Twist velo;
        if(distance<min_dis)
        {
            velo.angular.z =  0.0;
            velo.linear.x = 0.0;
        }

        else
        {
            velo.angular.z =  theta;
            velo.linear.x = linear;
        }
        if(theta>max_theta)
        {
            velo.angular.z = max_theta;
        }
        
        vel_pub.publish(velo);
    }
    else
    {
        geometry_msgs::Twist velo;
        velo.angular.z = 0.0;
        velo.linear.x = 0.0;
        vel_pub.publish(velo);

    }
        
}
void pos2vel::lost_cb(const std_msgs::BoolConstPtr &msg)
{
    lost_flag = msg->data;
    if(lost_flag == true)
    {
        ROS_INFO("Target has lost!!!!!!!");
    }
}
pos2vel::pos2vel(/* args */)
{
    nh.param<std::string>("postopic", postopic, "/mytarget_position");
    nh.param<std::string>("veltopic", veltopic, "/cmd_vel");
    nh.param<std::string>("lostopic", lost_flag_topic, "/lost_flag");

    nh.param<float>("kp",kp,0.2);
    nh.param<float>("kp_theta",kp_theta,2.0);
    nh.param<float>("max_theta",max_theta,pi/6);
    nh.param<float>("max_vel",max_vel,1.5);
    nh.param<float>("min_dis",min_dis,2.0);
    nh.param<float>("max_lost_time",max_lost_time,10);
    lost_sub = nh.subscribe<std_msgs::Bool>(lost_flag_topic, 10, &pos2vel::lost_cb, this);
    vel_pub = nh.advertise<geometry_msgs::Twist>(veltopic,10);
    pos_sub = nh.subscribe<geometry_msgs::PointStamped>(postopic,10,&pos2vel::pos_cb,this);
}

pos2vel::~pos2vel()
{
}

int main(int argc, char** argv) {
    ros::init(argc,argv,"pos2vel");

    pos2vel p2e;
    ros::spin();

    return 0;

}