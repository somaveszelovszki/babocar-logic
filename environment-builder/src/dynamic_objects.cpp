#include <babocar-core/types.hpp>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("map size: %d x %d", msg->info.width, msg->info.height);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "environment_builder__dynamic_objects");
    ros::NodeHandle n;
    ros::Rate rate(10);
    ros::Subscriber sub = n.subscribe("map", 1, occupancyGridCallback);

    while(ros::ok()) {
        ROS_INFO("Running");
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Program ended.");

    return 0;
}