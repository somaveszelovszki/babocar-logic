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
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <environment_builder/DynamicObjectArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <algorithm>
#include <regex>
#include <pthread.h>

using namespace bcr;

namespace {

class VoMapBuilderNode : public RosNode {
public:
    using RosNode::RosNode;

    tf::TransformListener transformListener;
};

constexpr millisecond_t RUN_PERIOD(180);

constexpr meter_t       CAR_RADIUS                            = centimeter_t(55);
constexpr meter_t       CAR_FRONT_REAR_WHEEL_AXIS_DIST        = centimeter_t(65);
constexpr m_per_sec_t   MAX_SPEED_BWD                         = m_per_sec_t(-1.0);
constexpr m_per_sec_t   MAX_SPEED_FWD                         = m_per_sec_t(1.0);
constexpr radian_t      MAX_WHEEL_ANGLE                       = radian_t(0.5);
constexpr m_per_sec2_t  MAX_ACCELERATION                      = m_per_sec2_t(0.5);
constexpr rad_per_sec_t MAX_WHEEL_ANGULAR_VELOCITY            = degree_t(60) / second_t(1.0);

constexpr m_per_sec_t   DYNAMIC_WINDOW_SPEED_RESOLUTION       = m_per_sec_t(0.05);
constexpr radian_t      DYNAMIC_WINDOW_WHEEL_ANGLE_RESOLUTION = degree_t(4);

constexpr meter_t       MAX_OBSTACLE_DISTANCE                 = meter_t(4);
constexpr millisecond_t COLLISION_CHECK_TIME_STEP             = millisecond_t(10);
constexpr meter_t       MIN_KEPT_DISTANCE                     = centimeter_t(10);

std::unique_ptr<VoMapBuilderNode> node = nullptr;

ros::Publisher *availableVelocitiesPub = nullptr;
ros::Publisher *steerPub = nullptr;
ros::Publisher *motorPub = nullptr;

DynamicObject car = {
    CAR_RADIUS,
    Odometry {
        Pose { { meter_t(0), meter_t(0) }, degree_t(90) },
        Twist { { m_per_sec_t(0), m_per_sec_t(0) }, rad_per_sec_t(0) }
    }
};
ros::Time carLastUpdateTime;
nav_msgs::Odometry carOdom_ros;

DynamicWindow window(
    MAX_SPEED_BWD, MAX_SPEED_FWD, MAX_WHEEL_ANGLE,
    MAX_ACCELERATION * RUN_PERIOD, MAX_WHEEL_ANGULAR_VELOCITY * RUN_PERIOD,
    DYNAMIC_WINDOW_SPEED_RESOLUTION, DYNAMIC_WINDOW_WHEEL_ANGLE_RESOLUTION
);

int32_t steer_in = 0;
int32_t steer_out = 0;

int32_t motor_in = 0;
int32_t motor_out = 0;

std::vector<DynamicObject> staticObjects;

nanosecond_t rosTimeDiff(const ros::Time& t1, const ros::Time& t2) {
    return nanosecond_t((t1 - t2).toNSec());
}

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

void sendAvailableVelocities(void) {
    visualization_msgs::MarkerArray availableVelocities;

    int32_t idx = 0;
    for (std::vector<VelocityObstacle>& vos : window.getWindow()) {
        for (VelocityObstacle& vo : vos) {
            if (vo.available && vo.speed != m_per_sec_t(0) && vo.safetyFactor > 0.0f) {
                Pose pose = car.odom.pose;
                pose.angle += vo.wheelAngle;

                visualization_msgs::Marker velo_marker;
                velo_marker.header.frame_id = "odom";
                velo_marker.header.stamp = ros::Time();
                velo_marker.ns = "bcr";
                velo_marker.id = ++idx;
                velo_marker.type = visualization_msgs::Marker::ARROW;
                velo_marker.action = visualization_msgs::Marker::ADD;
                velo_marker.pose = bcr::ros_convert(pose);
                velo_marker.scale.x = 5 * vo.safetyFactor;
                velo_marker.scale.y = 0.1;
                velo_marker.scale.z = 0.1;
                velo_marker.color.a = 1.0;
                velo_marker.color.r = 0.0;
                velo_marker.color.g = 0.0;
                velo_marker.color.b = 1.0;
                velo_marker.lifetime = ros::Duration(0.5);
                availableVelocities.markers.push_back(velo_marker);

                //ROS_INFO("%f m/s | %f deg", vo.speed.get(), static_cast<degree_t>(vo.wheelAngle).get());
            }
        }
    }

    availableVelocitiesPub->publish(availableVelocities);
}

void staticGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& staticGrid) {

    staticObjects.clear();
    staticObjects.reserve(staticGrid->data.size());

    const int32_t w = static_cast<int32_t>(staticGrid->info.width);
    const int32_t h = static_cast<int32_t>(staticGrid->info.height);

    for (int32_t i = 0; i < static_cast<int32_t>(staticGrid->data.size()); ++i) {
        if (staticGrid->data[i] > 50) {
            const Point2m absPoint = {
                meter_t(staticGrid->info.origin.position.x + ((i % w) - w / 2) * staticGrid->info.resolution),
                meter_t(staticGrid->info.origin.position.y + ((i / w) - h / 2) * staticGrid->info.resolution)
            };

            if (absPoint.distance(car.odom.pose.pos) < MAX_OBSTACLE_DISTANCE) {
                staticObjects.push_back({ centimeter_t(1), { Pose { absPoint, radian_t(0) }, Twist { { m_per_sec_t(0), m_per_sec_t(0) }, rad_per_sec_t(0) } } });
            }
        }
    }
}

void dynObjCallback(const environment_builder::DynamicObjectArray::ConstPtr& dynamicObjects) {

    car.odom.update(rosTimeDiff(ros::Time::now(), carLastUpdateTime));
    carLastUpdateTime = ros::Time::now();

    const radian_t desiredWheelAngle = bcr::map(static_cast<double>(steer_in), -500.0, 500.0, -MAX_WHEEL_ANGLE, MAX_WHEEL_ANGLE);
    const radian_t actualWheelAngle = bcr::map(static_cast<double>(steer_out), -500.0, 500.0, -MAX_WHEEL_ANGLE, MAX_WHEEL_ANGLE);

    const m_per_sec_t desiredSpeed = bcr::map(motor_in, -500, 500, m_per_sec_t(-1), m_per_sec_t(1));
    const m_per_sec_t actualSpeed = bcr::map(motor_out, -500, 500, m_per_sec_t(-1), m_per_sec_t(1));

    std::vector<DynamicObject> objects = staticObjects;
    objects.reserve(objects.size() + dynamicObjects->objects.size());

    ROS_INFO("----------------------------------------------------");
    for (const environment_builder::DynamicObject& o : dynamicObjects->objects) {
        DynamicObject obj = bcr::ros_convert(o);
        if (obj.odom.pose.pos.distance(car.odom.pose.pos) < MAX_OBSTACLE_DISTANCE) {
            objects.push_back(obj);
        }
    }

    ROS_INFO("Speed: %f (desired: %f) m/s | wheel angle: %f (desired: %f) deg",
        actualSpeed.get(), desiredSpeed.get(),
        static_cast<degree_t>(actualWheelAngle).get(), static_cast<degree_t>(desiredWheelAngle).get());

    window.update(actualSpeed, actualWheelAngle);

    for (std::vector<VelocityObstacle>& vos : window.getWindow()) {
        for (VelocityObstacle& vo : vos) {
            if (vo.available) {
                DynamicObject c = car;

                c.radius += MIN_KEPT_DISTANCE;
                c.odom.twist.speed *= vo.speed / actualSpeed;
                c.odom.twist.ang_vel = bcr::getAngularVelocity(CAR_FRONT_REAR_WHEEL_AXIS_DIST, vo.speed, vo.wheelAngle);

                const millisecond_t stopTime = abs(vo.speed) / MAX_ACCELERATION;
                const millisecond_t timeInterval = bcr::max(2 * stopTime, millisecond_t(1000));
                const millisecond_t collisionTime = bcr::getTimeToFirstCollision_iterative(c, objects, timeInterval, COLLISION_CHECK_TIME_STEP);

                vo.safetyFactor    = bcr::map(collisionTime, millisecond_t(0), timeInterval, 0.0f, 1.0f);
                vo.directionFactor = bcr::map(abs(vo.wheelAngle - desiredWheelAngle), radian_t(0), 2 * MAX_WHEEL_ANGLE, 1.0f, 0.0f);
                vo.speedFactor     = bcr::map(abs(vo.speed - desiredSpeed), m_per_sec_t(0), MAX_SPEED_FWD - MAX_SPEED_BWD, 1.0f, 0.0f);

                ROS_INFO("Update - VO: %f m/s | %f deg\t-> stopTime: %f sec | collisionTime: %f sec -> %f",
                    vo.speed.get(), static_cast<degree_t>(vo.wheelAngle).get(),
                    static_cast<second_t>(stopTime).get(), static_cast<second_t>(collisionTime).get(), vo.safetyFactor);
            } else {
                vo.safetyFactor = 0.0f;
                vo.directionFactor = 0.0f;
                vo.speedFactor = 0.0f;
            }
        }
    }

    VelocityObstacle velo = window.getWindow()[0][0];
    float32_t maxFactor = velo.safetyFactor * velo.directionFactor * velo.speedFactor;

    for (std::vector<VelocityObstacle>& vos : window.getWindow()) {
        for (VelocityObstacle& vo : vos) {
            const float32_t factor = vo.safetyFactor * vo.directionFactor * vo.speedFactor;

            ROS_INFO("Evaluate - VO: %f m/s | %f deg\t-> safety (%f) * dir (%f) * speed (%f) = %f%s",
                vo.speed.get(), static_cast<degree_t>(vo.wheelAngle).get(),
                vo.safetyFactor, vo.directionFactor, vo.speedFactor,
                factor,
                factor > maxFactor ? " -> new max" : "");

            if (factor > maxFactor) {
                velo = vo;
                maxFactor = factor;
            }
        }
    }

    if (maxFactor < 0.1f) {
        VelocityObstacle& velo = window.getWindow()[0][0];
        for (std::vector<VelocityObstacle>& vos : window.getWindow()) {
            for (VelocityObstacle& vo : vos) {
                const float32_t factor = vo.safetyFactor * vo.directionFactor;

                ROS_INFO("Evaluate - VO: %f m/s | %f deg\t-> safety (%f) * dir (%f) = %f%s",
                    vo.speed.get(), static_cast<degree_t>(vo.wheelAngle).get(),
                    vo.safetyFactor, vo.directionFactor,
                    factor,
                    factor > maxFactor ? " -> new max" : "");

                if (factor > maxFactor) {
                    velo = vo;
                    maxFactor = factor;
                }
            }
        }

        if (maxFactor < 0.2f) {
            VelocityObstacle& velo = window.getWindow()[0][0];
            for (std::vector<VelocityObstacle>& vos : window.getWindow()) {
                for (VelocityObstacle& vo : vos) {
                    const float32_t factor = vo.safetyFactor;

                    ROS_INFO("Evaluate - VO: %f m/s | %f deg\t-> safety (%f) = %f%s",
                        vo.speed.get(), static_cast<degree_t>(vo.wheelAngle).get(),
                        vo.safetyFactor,
                        factor,
                        factor > maxFactor ? " -> new max" : "");
    
                    if (factor > maxFactor) {
                        velo = vo;
                        maxFactor = factor;
                    }
                }
            }
            if (maxFactor < 0.5f) {
                // force stop
                velo.wheelAngle = actualWheelAngle;
                velo.speed = m_per_sec_t(0);
            }
        }
    }

    steer_out = bcr::map(velo.wheelAngle, -MAX_WHEEL_ANGLE, MAX_WHEEL_ANGLE, -500.0, 500.0);
    motor_out = bcr::map(velo.speed, MAX_SPEED_BWD, MAX_SPEED_FWD, -500.0, 500.0);

    sendAvailableVelocities();
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    static ros::Time prevTime = odom->header.stamp;
    static Odometry prevOdom = car.odom;

    const millisecond_t d_time = rosTimeDiff(odom->header.stamp, prevTime);

    carOdom_ros = *odom;
    car.odom = bcr::ros_convert(*odom);

    car.odom.twist.speed = (car.odom.pose.pos - prevOdom.pose.pos) / d_time;
    const m_per_sec_t actualSpeed = bcr::map(motor_out, -500, 500, m_per_sec_t(-1), m_per_sec_t(1));
    car.odom.twist.speed *= abs(actualSpeed) / car.odom.twist.speed.length();

    car.odom.twist.ang_vel = (car.odom.pose.angle - prevOdom.pose.angle) / d_time;

    prevTime = odom->header.stamp;
    prevOdom = car.odom;
    carLastUpdateTime = ros::Time::now();

     ROS_INFO("Odom: pos: [%f, %f] m, %f deg, twist: speed : [%f, %f] m/s, %f deg/s",
        car.odom.pose.pos.X.get(), car.odom.pose.pos.Y.get(), static_cast<degree_t>(car.odom.pose.angle).get(),
        car.odom.twist.speed.X.get(), car.odom.twist.speed.Y.get(), static_cast<deg_per_sec_t>(car.odom.twist.ang_vel).get());
}

void steerCallback(const std_msgs::String::ConstPtr& cmd) {

    const std::regex regex("SET STEER_REF (-?\\d+)/0");
    std::smatch match;
    if (std::regex_match(cmd->data, match, regex) && match.size() == 2) {
        steer_in = std::stoi(match[1]);
    }
}

void motorCallback(const std_msgs::String::ConstPtr& cmd) {

    const std::regex regex("SET PWM (-?\\d+)/0");
    std::smatch match;
    if (std::regex_match(cmd->data, match, regex) && match.size() == 2) {
        motor_in = std::stoi(match[1]);
    }
}

void* publishControlData(void *argument) {
    while (true) {
        std_msgs::String steer_msg;
        steer_msg.data = "SET STEER_REF " + std::to_string(steer_out) + " " + std::to_string(motor_out) + "/0";
        steerPub->publish(steer_msg);

        usleep(50000);
    }

    return nullptr;
}

} // namespace

int main(int argc, char **argv)
{
    static const std::string NODE_NAME = "local_planner__vo_map_builder";
    ros::init(argc, argv, NODE_NAME);
    node.reset(new VoMapBuilderNode(NODE_NAME));
    ros::Subscriber odomSub = node->subscribe<nav_msgs::Odometry>("odom", 1, odomCallback);
    ros::Subscriber staticGridSub = node->subscribe<nav_msgs::OccupancyGrid>("static_grid", 1, staticGridCallback);
    ros::Subscriber dynObjSub = node->subscribe<environment_builder::DynamicObjectArray>("dynamic_objects", 1, dynObjCallback);
    ros::Subscriber steerSub = node->subscribe<std_msgs::String>("steer_orig", 1, steerCallback);
    ros::Subscriber motorSub = node->subscribe<std_msgs::String>("motor_orig", 1, motorCallback);

    ros::Publisher availableVelocitiesPublisher = node->advertise<visualization_msgs::MarkerArray>("available_velocities", 10);
    availableVelocitiesPub = &availableVelocitiesPublisher;

    ros::Publisher steerPublisher = node->advertise<std_msgs::String>("steer", 10);
    steerPub = &steerPublisher;

    ros::Publisher motorPublisher = node->advertise<std_msgs::String>("motor", 10);
    motorPub = &motorPublisher;

    pthread_t t;
    pthread_create(&t, NULL, publishControlData, NULL);

    ros::spin();

    return 0;
}
