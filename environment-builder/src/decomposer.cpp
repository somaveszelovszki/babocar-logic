#include <babocar-core/numeric.hpp>
#include <babocar-core/container/ring_buffer.hpp>
#include <babocar-core/odometry.hpp>
#include <babocar-core/ros_node.hpp>
#include <environment-builder/dynamic_object.hpp>
#include <environment-builder/absolute_map.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <vector>
#include <algorithm>

using namespace bcr;

namespace {

class DecomposerNode : public RosNode {
public:
    using RosNode::RosNode;

    tf::TransformListener lidarTransformListener;
};

/* float32_t getDistance(const sensor_msgs::LaserScan::ConstPtr& scan, radian_t angle) {
    float32_t dist;

    if (angle.get() < scan->angle_min || angle.get() > scan->angle_max) {
        dist = 0.0f;
    } else {
        int32_t idx = bcr::round(scan->angle_increment / (angle - scan->angle_min));
        if (idx < 0) {
            idx += prev->ranges.size();
        }

        dist = scan->ranges[idx];
    }

    return dist;
} */

// TODO use tf

Odometry carOdom;
Pose lidarPose;


Point2<distance_t> rayDistToAbsPoint(const Odometry& carOdom, radian_t angle, meter_t dist) {
    return { carOdom.pose.X + bcr::cos(carOdom.pose.angle + angle) * dist, carOdom.pose.Y + bcr::sin(carOdom.pose.angle + angle) * dist };
}
} // namespace

std::unique_ptr<DecomposerNode> node = nullptr;

AbsoluteMap absMap;

ring_buffer<sensor_msgs::LaserScan::ConstPtr, 10> prevScans;

sensor_msgs::LaserScan staticScan;
sensor_msgs::LaserScan dynamicScan;

std::vector<DynamicObject> dynamicObjects;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int32_t count = scan->scan_time / scan->time_increment;
    ROS_INFO("Scan frame id: '%s', count: %d", scan->header.frame_id.c_str(), count);
  
    for(int32_t i = 0; i < count; i++) {
        radian_t rad = radian_t(scan->angle_min + scan->angle_increment * i);
        ROS_INFO(": [%f, %f]", static_cast<degree_t>(rad).get(), scan->ranges[i]);
    }

    if (prevScans.full()) { // checks if enough samples have been collected
        const sensor_msgs::LaserScan::ConstPtr& prev = prevScans[prevScans.size() - 1];

        for(int32_t i = 0; i < count; i++) {
            const radian_t angle = radian_t(scan->angle_min + scan->angle_increment * i);
            const float32_t dist = scan->ranges[i];
            const float32_t prevDist = prev->ranges[i];
            //const float32_t prevDist = getDistance(prev, radian_t(scan->angle_min + i * scan->angle_increment));

            // TODO get tf transform (lidar -> world)


            geometry_msgs::PointStamped laser_point;
            laser_point.header.frame_id = "scan";

            //we'll just use the most recent transform available for our simple example
            laser_point.header.stamp = ros::Time();

            //just an arbitrary point in space
            laser_point.point.x = dist * bcr::cos(angle);
            laser_point.point.y = dist * bcr::sin(angle);
            laser_point.point.z = 0.0;

            try{
                geometry_msgs::PointStamped base_point;
                node->lidarTransformListener.transformPoint("base_link", laser_point, base_point);

                ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                    laser_point.point.x, laser_point.point.y, laser_point.point.z,
                    base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());

                absMap.setRay(lidarPose, Point2<distance_t>(meter_t(base_point.point.x), meter_t(base_point.point.y)));
            }
            catch(tf::TransformException& ex){
                ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
            }

        }
    }

    prevScans.append(scan);
}

int main(int argc, char **argv)
{
    static const std::string NODE_NAME = "environment_builder__decomposer";
    ros::init(argc, argv, NODE_NAME);
    node.reset(new DecomposerNode(NODE_NAME));
    ros::Subscriber sub = node->subscribe<sensor_msgs::LaserScan>("/scan", 5, scanCallback);

    absMap.initialize(meter_t(10), centimeter_t(5));

    staticScan.ranges.reserve(400);
    dynamicScan.ranges.reserve(400);

    dynamicObjects.reserve(50);

    absMap.setRay(Point2<distance_t>(meter_t(0), meter_t(0)), Point2<distance_t>(meter_t(-2), meter_t(1)));

    while(ros::ok()) {
        ROS_INFO("Running");    
        ros::spin();
    }

    ROS_INFO("Program ended.");

    return 0;
}