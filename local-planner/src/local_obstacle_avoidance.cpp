#include <babocar-core/numeric.hpp>
#include <babocar-core/container/ring_buffer.hpp>
#include <babocar-core/container/vec.hpp>
#include <babocar-core/odometry.hpp>
#include <babocar-core/linalg.hpp>
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
#include <nav_msgs/Path.h>
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

static constexpr float32_t W_SAFETY_FACTOR    = 4.0f;
static constexpr float32_t W_DIRECTION_FACTOR = 1.0f;
static constexpr float32_t W_SPEED_FACTOR     = 2.0f;
 
static constexpr millisecond_t RUN_PERIOD(180);
 
static constexpr meter_t       CAR_LENGTH                            = centimeter_t(110);
static constexpr meter_t       CAR_WIDTH                             = centimeter_t(50);
static constexpr meter_t       CAR_FRONT_REAR_WHEEL_AXIS_DIST        = centimeter_t(65);
static constexpr m_per_sec_t   MAX_SPEED_BWD                         = m_per_sec_t(-1.0);
static constexpr m_per_sec_t   MAX_SPEED_FWD                         = m_per_sec_t(1.0);
static constexpr radian_t      MAX_WHEEL_ANGLE                       = radian_t(0.5);
static constexpr m_per_sec2_t  MAX_ACCELERATION                      = m_per_sec2_t(0.5);
static constexpr rad_per_sec_t MAX_WHEEL_ANGULAR_VELOCITY            = degree_t(60) / second_t(1.0);
 
static constexpr m_per_sec_t   DYNAMIC_WINDOW_SPEED_RESOLUTION       = m_per_sec_t(0.01);
static constexpr radian_t      DYNAMIC_WINDOW_WHEEL_ANGLE_RESOLUTION = degree_t(1);
 
static constexpr meter_t       MAX_OBSTACLE_DISTANCE                 = meter_t(5);
static constexpr millisecond_t COLLISION_CHECK_TIME_STEP             = millisecond_t(100);
static constexpr meter_t       MIN_KEPT_DISTANCE                     = centimeter_t(25);

static constexpr meter_t       TRAJECTORY_RESOLUTION                 = centimeter_t(10);

std::unique_ptr<VoMapBuilderNode> node = nullptr;

ros::Publisher *availableVelocitiesPub = nullptr;
ros::Publisher *steerPub = nullptr;
ros::Publisher *motorPub = nullptr;
ros::Publisher *globalTrajectoryPub = nullptr;

DynamicObject car(
    {
        { meter_t(0),                   CAR_WIDTH / 2 },
        { (CAR_LENGTH - CAR_WIDTH) / 2, CAR_WIDTH / 2 },
        { CAR_LENGTH - CAR_WIDTH,       CAR_WIDTH / 2 }
    },
    Odometry {
        Pose { { meter_t(0), meter_t(0) }, radian_t(0) },
        Twist { { m_per_sec_t(0), m_per_sec_t(0) }, rad_per_sec_t(0) }
    }
);
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

radian_t actualWheelAngle;
m_per_sec_t actualSpeed;

std::vector<StaticObject> staticObjects;

std::vector<Point2m> trajectory = {
    { meter_t(0.0), meter_t(0) },
    { meter_t(0.1), meter_t(0) },
    { meter_t(0.2), meter_t(0) },
    { meter_t(0.3), meter_t(0) },
    { meter_t(0.4), meter_t(0) },
    { meter_t(0.5), meter_t(0) },
    { meter_t(0.6), meter_t(0) },
    { meter_t(0.7), meter_t(0) },
    { meter_t(0.8), meter_t(0) },
    { meter_t(0.9), meter_t(0) },

    { meter_t(1.0), meter_t(0) },
    { meter_t(1.1), meter_t(0) },
    { meter_t(1.2), meter_t(0) },
    { meter_t(1.3), meter_t(0) },
    { meter_t(1.4), meter_t(0) },
    { meter_t(1.5), meter_t(0) },
    { meter_t(1.6), meter_t(0) },
    { meter_t(1.7), meter_t(0) },
    { meter_t(1.8), meter_t(0) },
    { meter_t(1.9), meter_t(0) },

    { meter_t(2.0), meter_t(0) },
    { meter_t(2.1), meter_t(0) },
    { meter_t(2.2), meter_t(0) },
    { meter_t(2.3), meter_t(0) },
    { meter_t(2.4), meter_t(0) },
    { meter_t(2.5), meter_t(0) },
    { meter_t(2.6), meter_t(0) },
    { meter_t(2.7), meter_t(0) },
    { meter_t(2.8), meter_t(0) },
    { meter_t(2.9), meter_t(0) },

    { meter_t(3.0), meter_t(0) },
    { meter_t(3.1), meter_t(0) },
    { meter_t(3.2), meter_t(0) },
    { meter_t(3.3), meter_t(0) },
    { meter_t(3.4), meter_t(0) },
    { meter_t(3.5), meter_t(0) },
    { meter_t(3.6), meter_t(0) },
    { meter_t(3.7), meter_t(0) },
    { meter_t(3.8), meter_t(0) },
    { meter_t(3.9), meter_t(0) },

    { meter_t(4.0), meter_t(0) },
    { meter_t(4.1), meter_t(-0.1) },
    { meter_t(4.2), meter_t(-0.2) },
    { meter_t(4.3), meter_t(-0.3) },
    { meter_t(4.4), meter_t(-0.4) },
    { meter_t(4.5), meter_t(-0.5) },
    { meter_t(4.6), meter_t(-0.6) },
    { meter_t(4.7), meter_t(-0.7) },
    { meter_t(4.8), meter_t(-0.8) },
    { meter_t(4.9), meter_t(-0.9) },

    { meter_t(5.0), meter_t(-1) },
    { meter_t(5.1), meter_t(-1.1) },
    { meter_t(5.2), meter_t(-1.2) },
    { meter_t(5.3), meter_t(-1.3) },
    { meter_t(5.4), meter_t(-1.4) },
    { meter_t(5.5), meter_t(-1.5) },
    { meter_t(5.6), meter_t(-1.6) },
    { meter_t(5.7), meter_t(-1.7) },
    { meter_t(5.8), meter_t(-1.8) },
    { meter_t(5.9), meter_t(-1.9) },

    { meter_t(6.0), meter_t(-2) },
    { meter_t(6.1), meter_t(-2.1) },
    { meter_t(6.2), meter_t(-2.2) },
    { meter_t(6.3), meter_t(-2.3) },
    { meter_t(6.4), meter_t(-2.4) },
    { meter_t(6.5), meter_t(-2.5) },
    { meter_t(6.6), meter_t(-2.6) },
    { meter_t(6.7), meter_t(-2.7) },
    { meter_t(6.8), meter_t(-2.8) },
    { meter_t(6.9), meter_t(-2.9) },

    { meter_t(7.0), meter_t(-3) },
    { meter_t(7.1), meter_t(-3.1) },
    { meter_t(7.2), meter_t(-3.2) },
    { meter_t(7.3), meter_t(-3.3) },
    { meter_t(7.4), meter_t(-3.4) },
    { meter_t(7.5), meter_t(-3.5) },
    { meter_t(7.6), meter_t(-3.5) },
    { meter_t(7.7), meter_t(-3.5) },
    { meter_t(7.8), meter_t(-3.5) },
    { meter_t(7.9), meter_t(-3.5) },

    { meter_t(8.0), meter_t(-3.5) },
    { meter_t(8.1), meter_t(-3.5) },
    { meter_t(8.2), meter_t(-3.5) },
    { meter_t(8.3), meter_t(-3.5) },
    { meter_t(8.4), meter_t(-3.5) },
    { meter_t(8.5), meter_t(-3.5) },
    { meter_t(8.6), meter_t(-3.5) },
    { meter_t(8.7), meter_t(-3.5) },
    { meter_t(8.8), meter_t(-3.5) },
    { meter_t(8.9), meter_t(-3.5) },

    { meter_t(9.0), meter_t(-3.5) },
    { meter_t(9.1), meter_t(-3.5) },
    { meter_t(9.2), meter_t(-3.5) },
    { meter_t(9.3), meter_t(-3.5) },
    { meter_t(9.4), meter_t(-3.5) },
    { meter_t(9.5), meter_t(-3.5) },
    { meter_t(9.6), meter_t(-3.5) },
    { meter_t(9.7), meter_t(-3.5) },
    { meter_t(9.8), meter_t(-3.5) },
    { meter_t(9.9), meter_t(-3.5) },

    { meter_t(10.0), meter_t(-3.5) },
    { meter_t(10.1), meter_t(-3.5) },
    { meter_t(10.2), meter_t(-3.5) },
    { meter_t(10.3), meter_t(-3.5) },
    { meter_t(10.4), meter_t(-3.5) },
    { meter_t(10.5), meter_t(-3.5) },
    { meter_t(10.6), meter_t(-3.5) },
    { meter_t(10.7), meter_t(-3.5) },
    { meter_t(10.8), meter_t(-3.5) },
    { meter_t(10.9), meter_t(-3.5) },

    { meter_t(11.0), meter_t(-3.5) },
    { meter_t(11.1), meter_t(-3.5) },
    { meter_t(11.2), meter_t(-3.5) },
    { meter_t(11.3), meter_t(-3.5) },
    { meter_t(11.4), meter_t(-3.5) },
    { meter_t(11.5), meter_t(-3.5) },
    { meter_t(11.6), meter_t(-3.5) },
    { meter_t(11.7), meter_t(-3.5) },
    { meter_t(11.8), meter_t(-3.5) },
    { meter_t(11.9), meter_t(-3.5) },

#define X 12.0
#define Y -1.5
#define R 2.0

    { meter_t(X + R * sin(0.00 * PI)), meter_t(Y - R * cos(0.00 * PI)) },
    { meter_t(X + R * sin(0.05 * PI)), meter_t(Y - R * cos(0.05 * PI)) },
    { meter_t(X + R * sin(0.10 * PI)), meter_t(Y - R * cos(0.10 * PI)) },
    { meter_t(X + R * sin(0.15 * PI)), meter_t(Y - R * cos(0.15 * PI)) },
    { meter_t(X + R * sin(0.20 * PI)), meter_t(Y - R * cos(0.20 * PI)) },
    { meter_t(X + R * sin(0.25 * PI)), meter_t(Y - R * cos(0.25 * PI)) },
    { meter_t(X + R * sin(0.30 * PI)), meter_t(Y - R * cos(0.30 * PI)) },
    { meter_t(X + R * sin(0.35 * PI)), meter_t(Y - R * cos(0.35 * PI)) },
    { meter_t(X + R * sin(0.40 * PI)), meter_t(Y - R * cos(0.40 * PI)) },
    { meter_t(X + R * sin(0.45 * PI)), meter_t(Y - R * cos(0.45 * PI)) },
    { meter_t(X + R * sin(0.50 * PI)), meter_t(Y - R * cos(0.50 * PI)) },
    { meter_t(X + R * sin(0.55 * PI)), meter_t(Y - R * cos(0.55 * PI)) },
    { meter_t(X + R * sin(0.60 * PI)), meter_t(Y - R * cos(0.60 * PI)) },
    { meter_t(X + R * sin(0.65 * PI)), meter_t(Y - R * cos(0.65 * PI)) },
    { meter_t(X + R * sin(0.70 * PI)), meter_t(Y - R * cos(0.70 * PI)) },
    { meter_t(X + R * sin(0.75 * PI)), meter_t(Y - R * cos(0.75 * PI)) },
    { meter_t(X + R * sin(0.80 * PI)), meter_t(Y - R * cos(0.80 * PI)) },
    { meter_t(X + R * sin(0.85 * PI)), meter_t(Y - R * cos(0.85 * PI)) },
    { meter_t(X + R * sin(0.90 * PI)), meter_t(Y - R * cos(0.90 * PI)) },
    { meter_t(X + R * sin(0.95 * PI)), meter_t(Y - R * cos(0.95 * PI)) },
    { meter_t(12), meter_t(0.5) },

#undef X
#undef Y
#undef R

    { meter_t(11.9), meter_t(0.5) },
    { meter_t(11.8), meter_t(0.5) },
    { meter_t(11.7), meter_t(0.5) },
    { meter_t(11.6), meter_t(0.5) },
    { meter_t(11.5), meter_t(0.5) },
    { meter_t(11.4), meter_t(0.5) },
    { meter_t(11.3), meter_t(0.5) },
    { meter_t(11.2), meter_t(0.5) },
    { meter_t(11.1), meter_t(0.5) },
    { meter_t(11.0), meter_t(0.5) },

    { meter_t(10.9), meter_t(0.5) },
    { meter_t(10.8), meter_t(0.5) },
    { meter_t(10.7), meter_t(0.5) },
    { meter_t(10.6), meter_t(0.5) },
    { meter_t(10.5), meter_t(0.5) },
    { meter_t(10.4), meter_t(0.5) },
    { meter_t(10.3), meter_t(0.5) },
    { meter_t(10.2), meter_t(0.5) },
    { meter_t(10.1), meter_t(0.5) },
    { meter_t(10.0), meter_t(0.5) },

    { meter_t(9.9), meter_t(0.5) },
    { meter_t(9.8), meter_t(0.5) },
    { meter_t(9.7), meter_t(0.5) },
    { meter_t(9.6), meter_t(0.5) },
    { meter_t(9.5), meter_t(0.5) },
    { meter_t(9.4), meter_t(0.5) },
    { meter_t(9.3), meter_t(0.5) },
    { meter_t(9.2), meter_t(0.5) },
    { meter_t(9.1), meter_t(0.5) },
    { meter_t(9.0), meter_t(0.5) },

    { meter_t(8.9), meter_t(0.5) },
    { meter_t(8.8), meter_t(0.5) },
    { meter_t(8.7), meter_t(0.5) },
    { meter_t(8.6), meter_t(0.5) },
    { meter_t(8.5), meter_t(0.5) },
    { meter_t(8.4), meter_t(0.5) },
    { meter_t(8.3), meter_t(0.5) },
    { meter_t(8.2), meter_t(0.5) },
    { meter_t(8.1), meter_t(0.5) },
    { meter_t(8.0), meter_t(0.5) }
};

Point2m destination;
radian_t desiredWheelAngle;
m_per_sec_t desiredSpeed;

ros::Time lastSteerCmdTime;

void updateDestination2() {
    static const size_t endIdx = trajectory.size() - 1;

    static double lineP = 1.0;
    static double lineD = 0.0;

    static meter_t prevDist(0);

    meter_t minDist = meter_t(std::numeric_limits<unit_storage_type>::infinity());
    size_t minIdx = 0;

    for (size_t i = 0; i < trajectory.size(); ++i) {
        const Point2m pos = trajectory[i];
        const meter_t dist = pos.distance(car.odom.pose.pos);
        if (dist < minDist) {
            minIdx = i;
            minDist = dist;
        }
    }

    if (minIdx > endIdx) minIdx = endIdx;
    const size_t neighbourIdx = minIdx == 0 ? 1 : trajectory[minIdx - 1].distance(car.odom.pose.pos) < trajectory[minIdx + 1].distance(car.odom.pose.pos) ? minIdx - 1 : minIdx + 1;

    const Line2d line(static_cast<Point2d>(trajectory[minIdx]), static_cast<Point2d>(trajectory[neighbourIdx]));
    const meter_t dist = meter_t(distance(line, static_cast<Point2d>(car.odom.pose.pos)));

    const meter_t distFromEnd = TRAJECTORY_RESOLUTION * (endIdx - minIdx);

    node->getParam("lineP", lineP);
    node->getParam("lineD", lineD);

    desiredWheelAngle = clamp(radian_t(dist.get() * lineP + (dist - prevDist).get() * lineD), -MAX_WHEEL_ANGLE, MAX_WHEEL_ANGLE);
    desiredSpeed = map(abs(desiredWheelAngle).get(), 0.0, MAX_WHEEL_ANGLE.get(), m_per_sec_t(1), m_per_sec_t(0.5)) * min(1.0, distFromEnd / meter_t(1.0));

    prevDist = dist;
}

void updateDestination() {
    static constexpr meter_t MIN_LOOK_AHEAD = meter_t(1.5);
    static constexpr meter_t MAX_LOOK_AHEAD = meter_t(2.5);

    static const size_t endIdx = trajectory.size() - 1;

    static bool initialized = false;
    if (!initialized) {
        Point2m pos = trajectory[trajectory.size() - 1];
        const Vec2m diff = pos - trajectory[trajectory.size() - 2];

        for (size_t i = 0; i < static_cast<size_t>(MAX_LOOK_AHEAD / TRAJECTORY_RESOLUTION); ++i) {
            pos += diff;
            trajectory.push_back({ pos });
        }

        initialized = true;
    }

    meter_t minDist = meter_t(std::numeric_limits<unit_storage_type>::infinity());
    size_t minIdx = 0;

    for (size_t i = 0; i < trajectory.size(); ++i) {
        const Point2m pos = trajectory[i];
        const meter_t dist = pos.distance(car.odom.pose.pos);
        if (dist < minDist) {
            minIdx = i;
            minDist = dist;
        }
    }

    if (minIdx > endIdx) minIdx = endIdx;

    meter_t maxDistWithCurrentWheelAngle(0);
    {
        DynamicObject c = car;
        const millisecond_t step(500);
        const size_t numSteps = 4;

        for (size_t i = 0; i <= numSteps; ++i) {
            meter_t minDistAtStep = meter_t(std::numeric_limits<unit_storage_type>::infinity());

            for (size_t j = 0; j < trajectory.size(); ++j) {
                const Point2m pos = trajectory[j];
                const meter_t dist = pos.distance(car.odom.pose.pos);
                if (dist < minDistAtStep) {
                    minDistAtStep = dist;
                }
            }

            if (minDistAtStep > maxDistWithCurrentWheelAngle) {
                maxDistWithCurrentWheelAngle = minDistAtStep;
            }

            c.update(step);
        }
    }

    destination = trajectory[minIdx + static_cast<size_t>(map(maxDistWithCurrentWheelAngle, meter_t(0), meter_t(0.3), MIN_LOOK_AHEAD, MAX_LOOK_AHEAD) / TRAJECTORY_RESOLUTION)];
    const meter_t distFromEnd = TRAJECTORY_RESOLUTION * (endIdx - minIdx);

    desiredWheelAngle = getTrajectoryWheelAngle(CAR_FRONT_REAR_WHEEL_AXIS_DIST, car.odom.pose.pos, car.odom.pose.angle, destination);
    desiredSpeed = map(abs(desiredWheelAngle).get(), 0.0, MAX_WHEEL_ANGLE.get(), m_per_sec_t(1), m_per_sec_t(0.5)) * min(1.0, distFromEnd / meter_t(1.0));
}

nanosecond_t rosTimeDiff(const ros::Time& t1, const ros::Time& t2) {
    return nanosecond_t((t1 - t2).toNSec());
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

void sendGlobalTrajectory(void) {
    static uint32_t seq = 0;
    ++seq;

    nav_msgs::Path path;

    path.header.seq = seq;
    path.header.stamp = ros::Time();
    path.header.frame_id = "map";

    for (const Point2m& pos : trajectory) {
        geometry_msgs::PoseStamped pose;

        pose.header.seq = seq;
        pose.header.stamp = ros::Time();
        pose.header.frame_id = "map";

        pose.pose.position.x = pos.X.get();
        pose.pose.position.y = pos.Y.get();
        pose.pose.position.z = 0;

        path.poses.push_back(pose);
    }

    globalTrajectoryPub->publish(path);
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

            bool ok = true;
            for (size_t r = 0; r < car.radiuses.size(); ++r) {
                if (car.positions[r].distance(absPoint) < car.radiuses[r].second) {
                    ok = false;
                    break;
                }
            }

            if (ok && absPoint.distance(car.odom.pose.pos) < MAX_OBSTACLE_DISTANCE) {
                staticObjects.push_back({ centimeter_t(1), absPoint });
            }
        }
    }
}

millisecond_t collisionCheckTimeInterval(millisecond_t stopTime) {
    return bcr::max(3 * stopTime, millisecond_t(3000));
}

void dynObjCallback(const environment_builder::DynamicObjectArray::ConstPtr& dynamicObjectArray) {

    car.odom.update(rosTimeDiff(ros::Time::now(), carLastUpdateTime));
    carLastUpdateTime = ros::Time::now();

    if (rosTimeDiff(ros::Time::now(), lastSteerCmdTime) > second_t(1)) {
        updateDestination();
    } else {
        desiredWheelAngle = bcr::map(static_cast<double>(steer_in), -500.0, 500.0, -MAX_WHEEL_ANGLE, MAX_WHEEL_ANGLE);
        desiredSpeed = bcr::map(motor_in, -500, 500, m_per_sec_t(-1), m_per_sec_t(1));
    }

    std::vector<ObjectTrajectory> dynamicObjectTrajectories;
    dynamicObjectTrajectories.reserve(dynamicObjectArray->objects.size());

    window.update(actualSpeed, actualWheelAngle);

    m_per_sec_t maxSpeed = m_per_sec_t(0);

    for (std::vector<VelocityObstacle>& vos : window.getWindow()) {
        for (VelocityObstacle& vo : vos) {
            if (vo.available) {
                if (abs(vo.speed) > maxSpeed) {
                    maxSpeed = abs(vo.speed);
                }
            }
        }
    }

    const millisecond_t maxStopTime = maxSpeed / MAX_ACCELERATION;

    ROS_INFO("----------------------------------------------------");
    for (const environment_builder::DynamicObject& o : dynamicObjectArray->objects) {
        DynamicObject obj = bcr::ros_convert(o);

        bool ok = true;
        for (size_t r = 0; r < car.radiuses.size(); ++r) {
            if (car.positions[r].distance(obj.odom.pose.pos) < car.radiuses[r].second + obj.radiuses[0].second) {
                ok = false;
                break;
            }
        }

        if (ok) {
            dynamicObjectTrajectories.push_back(getTrajectory(obj, collisionCheckTimeInterval(maxStopTime), COLLISION_CHECK_TIME_STEP));
        }
    }

    ROS_INFO("Speed: %f (desired: %f) m/s | wheel angle: %f (desired: %f) deg",
        actualSpeed.get(), desiredSpeed.get(),
        static_cast<degree_t>(actualWheelAngle).get(), static_cast<degree_t>(desiredWheelAngle).get());

    for (std::vector<VelocityObstacle>& vos : window.getWindow()) {
        for (VelocityObstacle& vo : vos) {
            if (vo.available) {
                DynamicObject c = car;
                const meter_t minKeptDist = MIN_KEPT_DISTANCE * map(abs(vo.speed), m_per_sec_t(0), MAX_SPEED_FWD, 0.2, 1.0);

                for (std::pair<meter_t, meter_t>& r : c.radiuses) {
                    r.second += minKeptDist;
                }

                c.odom.twist.speed = Vec2mps(vo.speed, m_per_sec_t(0)).rotate(c.odom.pose.angle);
                c.odom.twist.ang_vel = bcr::getAngularVelocity(CAR_FRONT_REAR_WHEEL_AXIS_DIST, vo.speed, vo.wheelAngle);

                const millisecond_t stopTime = abs(vo.speed) / MAX_ACCELERATION;
                const millisecond_t collisionTime = bcr::getTimeToFirstCollision_iterative(c, staticObjects, dynamicObjectTrajectories, collisionCheckTimeInterval(stopTime), COLLISION_CHECK_TIME_STEP);

                vo.safetyFactor    = bcr::map(collisionTime, millisecond_t(0), collisionCheckTimeInterval(stopTime), 0.0f, 1.0f);
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

    VelocityObstacle control, safestControl;
    safestControl.safetyFactor = 0.0f;
    float32_t maxFactor = 0.0f;

    for (std::vector<VelocityObstacle>& vos : window.getWindow()) {
        for (VelocityObstacle& vo : vos) {
            const float32_t factor = vo.getFactor(W_SAFETY_FACTOR, W_DIRECTION_FACTOR, W_SPEED_FACTOR);

            ROS_INFO("Evaluate - VO: %f m/s | %f deg\t-> safety (%f), dir (%f), speed (%f) = %f%s",
                vo.speed.get(), static_cast<degree_t>(vo.wheelAngle).get(),
                vo.safetyFactor, vo.directionFactor, vo.speedFactor,
                factor,
                factor > maxFactor ? " -> new max" : "");

            if (factor > maxFactor) {
                control = vo;
                maxFactor = factor;
            }

            if (vo.safetyFactor > safestControl.safetyFactor) {
                safestControl = vo;
            }
        }
    }

    if (control.safetyFactor < 0.25f) {
        ROS_INFO("Collision alert!");
        //control = safestControl;
    }

    steer_out = bcr::map(control.wheelAngle, -MAX_WHEEL_ANGLE, MAX_WHEEL_ANGLE, -500.0, 500.0);
    motor_out = bcr::map(control.speed, MAX_SPEED_BWD, MAX_SPEED_FWD, -500.0, 500.0);

    sendAvailableVelocities();
    sendGlobalTrajectory();
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    static ros::Time prevTime = odom->header.stamp;
    static Odometry prevOdom = car.odom;

    const millisecond_t d_time = rosTimeDiff(odom->header.stamp, prevTime);

    carOdom_ros = *odom;
    car.odom = bcr::ros_convert(*odom);

    actualWheelAngle = bcr::map(static_cast<double>(steer_out), -500.0, 500.0, -MAX_WHEEL_ANGLE, MAX_WHEEL_ANGLE);
    actualSpeed = bcr::map(motor_out, -500, 500, m_per_sec_t(-1), m_per_sec_t(1));

    //car.odom.twist.speed = (car.odom.pose.pos - prevOdom.pose.pos) / d_time;
    //car.odom.twist.speed *= abs(actualSpeed) / car.odom.twist.speed.length();
    car.odom.twist.speed = Vec2mps(actualSpeed, m_per_sec_t(0)).rotate(car.odom.pose.angle);

    //car.odom.twist.ang_vel = (car.odom.pose.angle - prevOdom.pose.angle) / d_time;
    car.odom.twist.ang_vel = getAngularVelocity(CAR_FRONT_REAR_WHEEL_AXIS_DIST, actualSpeed, actualWheelAngle);

    car.updateCirclePositions();

    prevTime = odom->header.stamp;
    prevOdom = car.odom;
    carLastUpdateTime = ros::Time::now();

     ROS_INFO("Odom: pos: [%f, %f] m, %f deg, twist: speed : [%f, %f] m/s, %f deg/s",
        car.odom.pose.pos.X.get(), car.odom.pose.pos.Y.get(), static_cast<degree_t>(car.odom.pose.angle).get(),
        car.odom.twist.speed.X.get(), car.odom.twist.speed.Y.get(), static_cast<deg_per_sec_t>(car.odom.twist.ang_vel).get());
}

void steerCallback(const std_msgs::String::ConstPtr& cmd) {

    lastSteerCmdTime = ros::Time::now();
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

    ros::Publisher globalTrajectoryPublisher = node->advertise<nav_msgs::Path>("globalTrajectory", 10);
    globalTrajectoryPub = &globalTrajectoryPublisher;

    pthread_t t;
    pthread_create(&t, NULL, publishControlData, NULL);

    ros::spin();

    return 0;
}
