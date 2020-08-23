#include "odom.h"
#include "ros/ros.h"

odom_ros::odom_ros()
{
    start(1000);
}

void odom_ros::process()
{
//    int argc=0;char **argv = nullptr;
//    ros::init(argc, argv, "odom");
//    ros::NodeHandle private_nh_("~");
//    ros::Subscriber sub = private_nh_.subscribe("/odom", 100,&odom_ros::odom_callback,this);
//    ros::spin();
}

