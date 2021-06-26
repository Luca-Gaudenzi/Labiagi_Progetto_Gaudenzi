#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/thread/thread.hpp>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
ros::Publisher vel_pub;
float vel_x;
float vel_y;
void Cmd_VelCallback(const geometry_msgs::Twist::ConstPtr& msg){
    //geometry_msgs::Twist msg_send;
    //msg_send.linear.x=-msg->linear.x;
    //msg_send.linear.y=-msg->linear.y;
    //vel_pub.publish(msg_send);
    vel_x=msg->linear.x;
    vel_y=msg->linear.y;
    //ROS_INFO("ricevuto\n");
}

void Laser_Callback(const sensor_msgs::LaserScan& laser_msg){
    ROS_INFO("ricevuto\n");
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