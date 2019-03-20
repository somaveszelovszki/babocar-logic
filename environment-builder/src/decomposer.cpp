#include <babocar-core/numeric.hpp>
#include <babocar-core/container/ring_buffer.hpp>
#include <babocar-core/container/vec.hpp>
#include <babocar-core/odometry.hpp>
#include <babocar-core/kmeans.hpp>
#include <babocar-core/ros/ros_node.hpp>
#include <babocar-core/ros/ros_convert.hpp>
#include <environment-builder/dynamic_object.hpp>
#include <environment-builder/absolute_map.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <algorithm>

using namespace bcr;

namespace {

constexpr distance_t MAP_SIZE = meter_t(10);
constexpr distance_t MAP_RES = centimeter_t(5);

class DecomposerNode : public RosNode {
public:
    using RosNode::RosNode;

    tf::TransformListener lidarTransformListener;
};

std::unique_ptr<DecomposerNode> node = nullptr;

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

Point2m rayDistToAbsPoint(radian_t angle, meter_t dist) {

    geometry_msgs::PointStamped laser_point;
    geometry_msgs::PointStamped base_point;

    laser_point.header.frame_id = "lidar_scan";
    laser_point.header.stamp = ros::Time();
    laser_point.point.x = dist.get() * bcr::cos(angle);
    laser_point.point.y = dist.get() * bcr::sin(angle);
    laser_point.point.z = 0.0;
    
    // try{
    //     node->lidarTransformListener.transformPoint("base_link", laser_point, base_point);
    //     ROS_INFO("lidar_scan: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
    //         laser_point.point.x, laser_point.point.y, laser_point.point.z,
    //         base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
    // }
    // catch(tf::TransformException& ex){
    //     ROS_ERROR("Received an exception trying to transform a point from \"lidar_scan\" to \"base_link\": %s", ex.what());
    // }

    // TODO use previous odometry
    // TODO remove this line:
    base_point = laser_point;

    return { meter_t(base_point.point.x), meter_t(base_point.point.y) };

    //return { carOdom.pose.X + bcr::cos(carOdom.pose.angle + angle) * dist, carOdom.pose.Y + bcr::sin(carOdom.pose.angle + angle) * dist };
}

static constexpr uint32_t MAX_SCAN_SAMPLES = 400;

} // namespace

AbsoluteMap absMap;

struct LaserMeas {
    Odometry odom;
    //bcr::vec<Point2m, MAX_SCAN_SAMPLES> points;
    std::vector<Point2i> points;
};

ring_buffer<LaserMeas, 5> prevScans;

sensor_msgs::LaserScan staticScan;
nav_msgs::OccupancyGrid diffGrid;

std::vector<DynamicObject> dynamicObjects;

ros::Publisher *diffGridPub = nullptr;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int32_t count = (scan->angle_max - scan->angle_min) / scan->angle_increment;

    LaserMeas meas;
    meas.odom = carOdom;
    meas.points.reserve(count);

    staticScan = *scan;

    const double theta = 0.0;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Vector3 origin(-static_cast<meter_t>(MAP_SIZE).get() / 2.0, -static_cast<meter_t>(MAP_SIZE).get() / 2.0, 0.0);
    transform.setOrigin(origin);
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "diff_grid"));
    
    diffGrid.header.frame_id = "diff_grid";
    diffGrid.header.stamp = scan->header.stamp;

    diffGrid.info.width = diffGrid.info.height = MAP_SIZE / MAP_RES;
    diffGrid.info.resolution = static_cast<meter_t>(MAP_RES).get();

    diffGrid.info.origin.position.x = 0.0;
    diffGrid.info.origin.position.y = 0.0;
    diffGrid.info.origin.position.z = 0.0;
    tf::quaternionTFToMsg(q, diffGrid.info.origin.orientation);

    diffGrid.info.map_load_time = diffGrid.header.stamp;

    diffGrid.data.resize(diffGrid.info.width * diffGrid.info.height);
    std::fill(diffGrid.data.begin(), diffGrid.data.end(), 0);

    ROS_INFO("------------------------------------------------");

    for(int32_t i = 0; i < count; i++) {
        const radian_t angle = radian_t(scan->angle_min + scan->angle_increment * i);
        const meter_t dist = meter_t(scan->ranges[i]);

        // filters out too near objects (false detection)
        if (dist > centimeter_t(20) && !std::isinf(dist.get())) {
            const Point2m absPoint = rayDistToAbsPoint(angle, dist);
            //meas.points.push_back(absPoint);
            if (absMap.isInside(absPoint)) {
                const Point2i absMapPoint = absMap.getNearestIndexes(absPoint);
                meas.points.push_back(absMapPoint);  // stores quantated point
                ROS_INFO("%f, %f -> %d, %d", absPoint.X.get(), absPoint.Y.get(), absMapPoint.X, absMapPoint.Y);
            }
            
        }
    }

    if (prevScans.size() >= 1) { // checks if enough samples have been collected
        const LaserMeas& prev = prevScans[prevScans.size() - 1];
        // TODO prevprev and so on... (num of prevs as parameter)

        for (uint32_t i = 0; i < meas.points.size(); ++i) {
            const Point2i p = meas.points[i];
            bool found = false;
            for (uint32_t j = 0; j < prev.points.size(); ++j) {
                if (p == prev.points[j]) {
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                diffGrid.data[p.Y * diffGrid.info.width + p.X] = 100;
            }
        }

        //KMeans massCenters(meas.points);
        //uint32_t numObstacles = 5;  // TODO
        //const std::vector<KMeans::Group> groups = massCenters.run(numObstacles);
    }

    prevScans.append(meas);

    diffGridPub->publish(diffGrid);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    carOdom = bcr::ros_convert(*odom);
    ROS_INFO("Car pos: [%f, %f] m, orientation: %f deg", static_cast<meter_t>(carOdom.pose.X).get(), static_cast<meter_t>(carOdom.pose.Y).get(), static_cast<radian_t>(carOdom.pose.angle).get());
}

int main(int argc, char **argv)
{
    static const std::string NODE_NAME = "environment_builder__decomposer";
    ros::init(argc, argv, NODE_NAME);
    node.reset(new DecomposerNode(NODE_NAME));
    ros::Subscriber scanSub = node->subscribe<sensor_msgs::LaserScan>("lidar_scan", 1, scanCallback);
    ros::Subscriber odomSub = node->subscribe<nav_msgs::Odometry>("odom", 1, odomCallback);

    ros::Publisher diffGridPublisher = node->advertise<nav_msgs::OccupancyGrid>("diff_grid", 10);
    diffGridPub = &diffGridPublisher;

    absMap.initialize(MAP_SIZE, MAP_RES);

    staticScan.ranges.reserve(MAX_SCAN_SAMPLES);

    dynamicObjects.reserve(50);

    ros::spin();

    return 0;
}