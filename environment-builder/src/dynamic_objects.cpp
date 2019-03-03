#include <babocar-core/container/ring_buffer.hpp>
#include <environment-builder/dynamic_object.hpp>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <vector>
#include <algorithm>

using namespace bcr;

ring_buffer<nav_msgs::OccupancyGrid::ConstPtr, 10> mapSamples;
std::vector<DynamicObject> dynamicObjects;
std::vector<int8_t> occupancyDiff;

void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("map size: %d x %d", msg->info.width, msg->info.height);

    if (!mapSamples.empty() && (mapSamples[0]->info.width != msg->info.width || mapSamples[0]->info.height != msg->info.height || mapSamples[0]->info.resolution != msg->info.resolution)) {
        throw std::runtime_error("Map dimensions do not match! TODO handle different occupancy grid dimensions");
    }

    if (occupancyDiff.capacity() < msg->data.size()) {
        occupancyDiff.reserve(msg->data.size());
        std::fill(occupancyDiff.begin(), occupancyDiff.end(), 0);
    }

    if (mapSamples.full()) {
        // enough map samples have been collected
        
        nav_msgs::OccupancyGrid::_data_type::const_iterator itNew = msg->data.begin(), itPrev = mapSamples[mapSamples.size() - 1]->data.begin();
        nav_msgs::OccupancyGrid::_data_type::iterator itRes = occupancyDiff.begin();

        for (; itNew != msg->data.end(); ++itNew, ++itPrev, ++itRes) {
            if (*itNew > 100 || *itNew < 0 || *itPrev > 100 || *itPrev < 0) {   // filters out invalid values
                *itRes = 0;
            } else {
                // calculates difference of occupancy probabilities of the 2 different samples at the given position to get moving pixels,
                // and discretizes output values
                *itRes = (static_cast<int32_t>(*itNew) - *itPrev) / 100 > 50 ? 100 : 0;
            }
        }
    }

    mapSamples.append(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "environment_builder__static_points");
    ros::NodeHandle n;
    ros::Rate rate(1);
    ros::Subscriber sub = n.subscribe("map", 1, occupancyGridCallback);

    dynamicObjects.reserve(50);

    while(ros::ok()) {
        ROS_INFO("Running");
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Program ended.");

    return 0;
}