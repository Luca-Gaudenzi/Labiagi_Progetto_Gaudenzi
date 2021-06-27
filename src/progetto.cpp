#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/thread/thread.hpp>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include "tf/transform_listener.h"
#include "progetto/Prog_Cmd_vel.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

ros::Publisher vel_pub;
float vel_x;
float vel_y;
bool continua=false;

void Prog_Cmd_vel_Callback(const progetto::Prog_Cmd_vel::ConstPtr& msg){
    ROS_INFO("entrato %f %f\n", msg->vx, msg->vy);
    continua=true;
    vel_x=msg->vx;
    vel_y=msg->vy;
}

void Laser_Callback(const sensor_msgs::LaserScan::ConstPtr& laser_msg){
    if(continua==false) return;
    continua=false;
    
    sensor_msgs::PointCloud cloud;
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;
    projector.transformLaserScanToPointCloud("base_laser_link", *laser_msg, cloud, listener);
    
    //ROS_INFO("ricevuto:\n");
    //ciclo for
    int i=0;
    float sum_x=0;
    float sum_y=0;
    for(auto& point: cloud.points){
        float modulo= 1/sqrt(point.x*point.x+point.y*point.y);
        sum_x+=point.x*modulo*modulo;
        sum_y+=point.y*modulo*modulo;
    }
    //sum_y=0;
    //sum_x=0;
    
    //if(sum_x<250 && sum_x>-250) sum_x=0;
    //if(sum_y<250 && sum_y>-250) sum_y=0;
    geometry_msgs::Twist msg_send;
    msg_send.linear.x=-sum_x+vel_x;

    msg_send.linear.y=-sum_y+vel_y;
    ROS_INFO("%f %f\n", msg_send.linear.x, msg_send.linear.y);
    vel_pub.publish(msg_send);
}
int main(int argc, char **argv){
    /*if(argc<2) return -1;
    sscanf(argv[1], "%f", &vel_x);
    sscanf(argv[2], "%f", &vel_y);
    ROS_INFO("I COMANDI IN INPUT: %f %f", vel_x, vel_y);*/
    ros::init(argc, argv, "progetto");
    
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    
    //ros::Subscriber vel_sub= n.subscribe("cmd_vel", 1000, Cmd_VelCallback );
    vel_pub=n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

    ros::Subscriber laser_sub= n.subscribe("/base_scan", 1000, Laser_Callback);
    ros::Subscriber vel_sub=n.subscribe("/Prog_Cmd_vel", 1000, Prog_Cmd_vel_Callback);
    //tf2_ros::TransformListener tfListener(tfBuffer);
    //while(continua)
      //  ros::spinOnce();
    /*while(ros::ok() && continua){
        loop_rate.sleep();
        ros::spinOnce();
    }*/
    ros::spin();
    return 0;
}
