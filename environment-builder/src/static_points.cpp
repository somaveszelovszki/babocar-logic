#include <babocar-core/container/ring_buffer.hpp>
#include <babocar-core/ros_node.hpp>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace bcr;

const std::string PARAM_OCCUPANCY_LIMIT = "occupancy_limit";

std::unique_ptr<RosNode> node = nullptr;

ring_buffer<nav_msgs::OccupancyGrid::ConstPtr, 10> mapSamples;
nav_msgs::OccupancyGrid staticGrid;

void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("map size: %d x %d", msg->info.width, msg->info.height);

    if (!mapSamples.empty() && (mapSamples[0]->info.width != msg->info.width || mapSamples[0]->info.height != msg->info.height || mapSamples[0]->info.resolution != msg->info.resolution)) {
        throw std::invalid_argument("Map dimensions do not match! TODO handle different occupancy grid dimensions");
    }

    staticGrid.header = msg->header;
    staticGrid.info = msg->info;

    if (mapSamples.full()) {
        // enough map samples have been collected
        if (staticGrid.data.capacity() < msg->data.size()) {
            staticGrid.data.reserve(msg->data.size());
        }

        nav_msgs::OccupancyGrid::_data_type::const_iterator itNew = msg->data.begin(), itPrev = mapSamples[mapSamples.size() - 1]->data.begin();
        nav_msgs::OccupancyGrid::_data_type::iterator itRes = staticGrid.data.begin();

        const int32_t occupancyLimit = node->getParameter<int32_t>(PARAM_OCCUPANCY_LIMIT);

        for (; itNew != msg->data.end(); ++itNew, ++itPrev, ++itRes) {
            if (*itNew > 100 || *itNew < 0 || *itPrev > 100 || *itPrev < 0) {   // filters out invalid values
                *itRes = 0;
            } else {
                // multiplies occupancy probabilities of the 2 different samples at the given position to filter out moving objects,
                // and discretizes output values
                *itRes = (static_cast<int32_t>(*itNew) * *itPrev) / 100 > occupancyLimit ? 100 : 0;
            }
        }

    } else {
        staticGrid.data = msg->data;
    }

    mapSamples.append(msg);
}

int main(int argc, char **argv)
{
    static const std::string NODE_NAME = "environment_builder__static_points";
    ros::init(argc, argv, NODE_NAME);
    node.reset(new RosNode(NODE_NAME, millisecond_t(100)));

    ros::Subscriber sub_occupancyGrid = node->subscribe("map", 1, occupancyGridCallback);
    ros::Publisher pub_staticMap = node->advertise<nav_msgs::OccupancyGrid>("static_map", 1);

    while(ros::ok()) {
        ROS_INFO("Running");
        ros::spinOnce();
        pub_staticMap.publish(staticGrid);
        node->rate.sleep();
    }

    ROS_INFO("Program ended.");

    return 0;
}