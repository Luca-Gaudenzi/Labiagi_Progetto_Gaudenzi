#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/thread/thread.hpp>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include "tf/transform_listener.h"

#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

ros::Publisher vel_pub;
float vel_x;
float vel_y;



void Cmd_VelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    
    vel_x=msg->linear.x;
    vel_y=msg->linear.y;
}

void Laser_Callback(const sensor_msgs::LaserScan::ConstPtr& laser_msg){
    sensor_msgs::PointCloud cloud;
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;
    projector.transformLaserScanToPointCloud("base_laser_link", *laser_msg, cloud, listener);
    int len=sizeof(cloud.points)/sizeof(cloud.points[0]);
    //ROS_INFO("%d\n", size(cloud.points) );
    
    ROS_INFO("ricevuto:\n");
    //ciclo for
    int i=0;
    float sum_x=0;
    float sum_y=0;
    for(auto& point: cloud.points){
        float modulo= 1/sqrt(point.x*point.x+point.y*point.y);
        sum_x+=cloud.points[i].x/(modulo*modulo);
        sum_y+=cloud.points[i].y/(modulo*modulo);
    }
    
    
    
    geometry_msgs::Twist msg_send;
    msg_send.linear.x=sum_x+vel_x;
    msg_send.linear.y=sum_y+vel_y;
    vel_pub.publish(msg_send);
    //sum_x+=(cloud.points[0].x*cloud.points[0].x)+(cloud.points[0].y*cloud.points[0].y);
}
int main(int argc, char **argv){
    ros::init(argc, argv, "progetto");
    
    ros::NodeHandle n;
    ros::Rate loop_rate(100);

    ros::Subscriber vel_sub= n.subscribe("cmd_vel", 1000, Cmd_VelCallback );
    vel_pub=n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    ros::Subscriber laser_sub= n.subscribe("/base_scan", 1000, Laser_Callback);
    //tf2_ros::TransformListener tfListener(tfBuffer);
    ros::spin();
    return 0;
}