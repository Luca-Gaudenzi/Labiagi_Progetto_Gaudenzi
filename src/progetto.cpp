#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/thread/thread.hpp>

#include <sstream>


int main(int argc, char **argv){
    ros::init(argc, argv, "progetto.cpp");

    ros::NodeHandle n;
    
    ros::Rate loop_rate(100);

    tf2_ros::TransformListener tfListener(tfBuffer);
    return 0;
}