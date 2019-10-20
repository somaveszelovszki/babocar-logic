#include <babocar-core/numeric.hpp>
#include <babocar-core/container/ring_buffer.hpp>
#include <babocar-core/container/vec.hpp>
#include <babocar-core/odometry.hpp>
#include <babocar-core/ros/ros_node.hpp>
#include <babocar-core/unit_utils.hpp>
#include <babocar-core/ros/ros_convert.hpp>
#include <environment-builder/dynamic_object.hpp>
#include <environment-builder/ros_convert.hpp>
#include <local-planner/collision.hpp>
#include <local-planner/dynamic_window.hpp>
#include <local-planner/trajectory.hpp>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <environment_builder/DynamicObjectArray.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <algorithm>

using namespace bcr;

namespace {

class VoMapBuilderNode : public RosNode {
public:
    using RosNode::RosNode;

    tf::TransformListener transformListener;
};

std::unique_ptr<VoMapBuilderNode> node = nullptr;

static constexpr millisecond_t RUN_PERIOD(200);

static constexpr centimeter_t CAR_RADIUS(25);
static constexpr centimeter_t CAR_FRONT_REAR_WHEEL_AXIS_DIST(35);
static constexpr m_per_sec_t MAX_SPEED_BWD(0.5);
static constexpr m_per_sec_t MAX_SPEED_FWD(1.0);
static constexpr degree_t MAX_WHEEL_ANGLE(30);
static constexpr m_per_sec2_t MAX_ACCELERATION(0.5);
static constexpr deg_per_sec_t MAX_WHEEL_ANGULAR_VELOCITY = degree_t(60) / millisecond_t(200);

static constexpr m_per_sec_t DYNAMIC_WINDOW_SPEED_RESOLUTION(0.1);
static constexpr degree_t DYNAMIC_WINDOW_WHEEL_ANGLE_RESOLUTION(1);

static DynamicObject car = {
    CAR_RADIUS,
    Odometry {
        Pose { { meter_t(0), meter_t(0) }, degree_t(90) },
        Twist { { m_per_sec_t(0), m_per_sec_t(0) }, rad_per_sec_t(0) }
    }
};

static DynamicWindow window(
    MAX_SPEED_BWD, MAX_SPEED_FWD, MAX_WHEEL_ANGLE,
    MAX_ACCELERATION * RUN_PERIOD, MAX_WHEEL_ANGULAR_VELOCITY * RUN_PERIOD,
    DYNAMIC_WINDOW_SPEED_RESOLUTION, DYNAMIC_WINDOW_WHEEL_ANGLE_RESOLUTION
);

Pose transformPose(const std::string& from_frame, const std::string& to_frame, const Pose& p, const ros::Time& time = ros::Time()) {

    geometry_msgs::PoseStamped from_pose;
    geometry_msgs::PoseStamped to_pose;

    from_pose.header.frame_id = from_frame;
    from_pose.header.stamp = time;
    from_pose.pose = bcr::ros_convert(p);
    
    try{
        node->transformListener.transformPose(to_frame, from_pose, to_pose);
        //ROS_INFO("scanner: (%.2f, %.2f. %.2f) -----> %s: (%.2f, %.2f, %.2f) at time %.2f",
        //     to_frame.c_str(),
        //     from_pose.point.x, from_pose.point.y, from_pose.point.z,
        //     to_pose.point.x, to_pose.point.y, to_pose.point.z, to_pose.header.stamp.toSec());
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a pose from \"%s\" to \"%s\": %s", from_frame.c_str(), to_frame.c_str(), ex.what());
    }

    return bcr::ros_convert(to_pose.pose);
}

Point2m transformPoint(const std::string& from_frame, const std::string& to_frame, const Point2m& p, const ros::Time time = ros::Time()) {

    geometry_msgs::PointStamped from_point;
    geometry_msgs::PointStamped to_point;

    from_point.header.frame_id = from_frame;
    from_point.header.stamp = time;
    from_point.point.x = p.X.get();
    from_point.point.y = p.Y.get();
    from_point.point.z = 0.0;
    
    try{
        node->transformListener.transformPoint(to_frame, from_point, to_point);
        //ROS_INFO("scanner: (%.2f, %.2f. %.2f) -----> %s: (%.2f, %.2f, %.2f) at time %.2f",
        //     to_frame.c_str(),
        //     from_point.point.x, from_point.point.y, from_point.point.z,
        //     to_point.point.x, to_point.point.y, to_point.point.z, to_point.header.stamp.toSec());
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from \"%s\" to \"%s\": %s", from_frame.c_str(), to_frame.c_str(), ex.what());
    }

    return { meter_t(to_point.point.x), meter_t(to_point.point.y) };
}

void dynObjCallback(const environment_builder::DynamicObjectArray::ConstPtr& dynamicObjects) {
    ROS_INFO("----------------------------------------------------");
    for (const environment_builder::DynamicObject& o : dynamicObjects->objects) {
        DynamicObject obj = bcr::ros_convert(o);
        ROS_INFO("Object: pos: (%f, %f), speed: (%f, %f)", obj.odom.pose.pos.X.get(), obj.odom.pose.pos.Y.get(), obj.odom.twist.speed.X.get(), obj.odom.twist.speed.Y.get());
    }

    const m_per_sec_t speed = car.odom.twist.speed.length();
    window.update(speed, bcr::getWheelAngle(CAR_FRONT_REAR_WHEEL_AXIS_DIST, speed, car.odom.twist.ang_vel));
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    car.odom = bcr::ros_convert(*odom);
    //ROS_INFO("Car pos: [%f, %f] m, orientation: %f deg", static_cast<meter_t>(carOdom.pose.pos.X).get(), static_cast<meter_t>(carOdom.pose.pos.Y).get(), static_cast<radian_t>(carOdom.pose.angle).get());
}

} // namespace

int main(int argc, char **argv)
{
    static const std::string NODE_NAME = "local_planner__vo_map_builder";
    ros::init(argc, argv, NODE_NAME);
    node.reset(new VoMapBuilderNode(NODE_NAME));
    ros::Subscriber odomSub = node->subscribe<nav_msgs::Odometry>("odom", 1, odomCallback);
    ros::Subscriber dynObjSub = node->subscribe<environment_builder::DynamicObjectArray>("dynamic_objects", 1, dynObjCallback);

    //ros::Publisher diffGridPublisher = node->advertise<nav_msgs::OccupancyGrid>("diff_grid", 10);
    //diffGridPub = &diffGridPublisher;

    ros::spin();

    return 0;
}
