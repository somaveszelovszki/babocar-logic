#include <babocar-core/numeric.hpp>
#include <babocar-core/container/ring_buffer.hpp>
#include <babocar-core/container/vec.hpp>
#include <babocar-core/odometry.hpp>
#include <babocar-core/kmeans.hpp>
#include <babocar-core/ros/ros_node.hpp>
#include <babocar-core/ros/ros_convert.hpp>
#include <environment-builder/dynamic_object.hpp>
#include <environment-builder/absolute_map.hpp>
#include <babocar-core/unit_utils.hpp>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <environment_builder/DynamicObjectArray.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <algorithm>

#define SINGLE_LIDAR true

using namespace bcr;

namespace {

constexpr meter_t LIDAR_MAX_DIST = meter_t(16.0f);
constexpr meter_t MAP_SIZE = meter_t(20);
constexpr meter_t MAP_RES = centimeter_t(10);

class DecomposerNode : public RosNode {
public:
    using RosNode::RosNode;

    tf::TransformListener transformListener;
};

std::unique_ptr<DecomposerNode> node = nullptr;

constexpr meter_t CAR_RADIUS = centimeter_t(55);

nav_msgs::Odometry carOdom_ros;
Odometry carOdom;

constexpr uint32_t MAX_SCAN_SAMPLES = 400;

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

AbsoluteMap absMap;

struct AbsPoint {
    enum class State {
        UNKNOWN,
        STATIC,
        DYNAMIC_NEG,    // negative-dynamic point (distant point visible after closer object has passed)
        DYNAMIC_POS     // positive-dynamic point (object passing in front of a distant point)
    };

    int32_t idx;
    meter_t dist;
    Point2m absMapPos;
    State state;
};

struct LaserMeas {
    sensor_msgs::LaserScan::ConstPtr scan;
    millisecond_t time;
    int32_t count;
    Odometry odom;
    Pose mapPose;
    std::vector<AbsPoint> points;
    
    LaserMeas(const sensor_msgs::LaserScan::ConstPtr& scan)
        : scan(scan)
        , time(second_t(scan->header.stamp.toSec()))
        , count((scan->angle_max - scan->angle_min) / scan->angle_increment) {

        this->points.reserve(count);
    }

    meter_t getDistance(radian_t angle) const {
        meter_t dist;

        if (angle.get() < this->scan->angle_min || angle.get() > this->scan->angle_max) {
            dist = distance_t::ZERO();
        } else {
            int32_t idx = static_cast<int32_t>((angle.get() - this->scan->angle_min) / this->scan->angle_increment);    // floor of index
            while (idx < 0) {
                idx += this->count;
            }

            while (idx >= this->count) {
                idx -= this->count;
            }

            dist = meter_t(scan->ranges[idx]);
        }

        return dist;
    }

    meter_t getDistanceInDirection(const Point2m& odomPoint) const {
#if SINGLE_LIDAR
        const Point2m lidarPoint = transformPoint("odom", "scanner", odomPoint, this->scan->header.stamp);
#else
        const Point2m lidarPoint = transformPoint("odom", "front_scanner", odomPoint, this->scan->header.stamp);//.average(transformPoint("odom", "rear_scanner", odomPoint, this->scan->header.stamp));
#endif

        return this->getDistance(atan2(lidarPoint.Y, lidarPoint.X));
    }
};

struct Group {
    static constexpr int32_t INVALID_IDX = -1;
    int32_t idx;
    uint32_t existingCntr;
    uint32_t forcedCntr;    // indicates if group has not been detected in the current measurement, but has been forced to exist (filtering)
    Point2m center;
    meter_t radius;
    std::vector<AbsPoint*> points;
    Vec2<m_per_sec_t> speed;
    bool isMoving_;

    Group()
        : idx(INVALID_IDX)
        , existingCntr(0)
        , forcedCntr(0)
        , isMoving_(false) {}

    static Group createFromPrev(const Group& prev, millisecond_t dt) {
        Group result;
        result.idx = prev.idx;
        result.center = prev.center + prev.speed * dt;
        result.radius = prev.radius;
        result.existingCntr = prev.existingCntr + 1;
        result.forcedCntr = prev.forcedCntr + 1;
        result.speed = prev.speed;
        result.isMoving_ = prev.isMoving_;
        return result;
    }

    void updateIsMoving() {
        this->isMoving_ = std::count_if(this->points.begin(), this->points.end(), [](const AbsPoint *p) { return p->state == AbsPoint::State::DYNAMIC_POS; }) >= 2;
    }

    bool isMoving() const {
        return this->isMoving_;
    }

    void merge(Group& other) {
        this->center = avg(this->center, other.center);
        this->radius = avg(this->radius, other.radius);
        this->forcedCntr = min(this->forcedCntr, other.forcedCntr);

        if (this->existingCntr < other.existingCntr) {
            this->idx = other.idx;
            this->existingCntr = other.existingCntr;
        }

        for (AbsPoint * const p : other.points) {
            if (std::find(this->points.begin(), this->points.end(), p) == this->points.end()) {
                this->points.push_back(p);
            }
        }
    }
};

constexpr uint32_t NUM_PREV_COMPARE = 5;
constexpr uint32_t NUM_SPEED_FILTER = 5;

ring_buffer<LaserMeas, NUM_PREV_COMPARE + 1> prevScans;
ring_buffer<std::vector<Group>, NUM_PREV_COMPARE + 1> prevGroupsList;

sensor_msgs::LaserScan staticScan;
nav_msgs::OccupancyGrid diffGrid;

std::vector<DynamicObject> dynamicObjects;

ros::Publisher *diffGridPub = nullptr;
ros::Publisher *staticGridPub = nullptr;
ros::Publisher *staticScanPub = nullptr;
ros::Publisher *dynamicObjectsPub = nullptr;
ros::Publisher *obstaclesPub = nullptr;

int32_t maxGroupIdx = Group::INVALID_IDX;

std::vector<Group> makeGroups(LaserMeas& meas, radian_t ray_angle_incr, const std::vector<radian_t>& negDiffAngles) {
    std::vector<Group> groups;

    if (meas.points.size() >= 2) {

        std::vector<Group>::iterator currentGroup = groups.end();

        const AbsPoint *p_prev = &meas.points[0];
        int32_t startIdx = 1;
        int32_t i = 1;

        do {
            AbsPoint& p = meas.points[i];
            const meter_t eps = bcr::clamp(meter_t(p.absMapPos.length() * ray_angle_incr * 4), meter_t(0.2f), meter_t(0.4f));

            // if point is far from its neighbour, we start a new group (new object)
            if (p.absMapPos.distance(p_prev->absMapPos) > eps) {
                // saves index of the first point thas has already been added to a group
                if (currentGroup == groups.end()) {
                    startIdx = i;
                } else {

                    // cuts off very long tails, that are probably walls very close to a moving objects
                    std::vector<AbsPoint*>::iterator firstDynamic = std::find_if(
                        currentGroup->points.begin(), currentGroup->points.end(),
                        [](const AbsPoint *p) { return p->state == AbsPoint::State::DYNAMIC_POS; });
                    bool beginningStaticPoints = firstDynamic != currentGroup->points.end();

                    std::vector<AbsPoint*>::iterator firstStaticAfterLastDynamic = std::find_if(
                        currentGroup->points.rbegin(), currentGroup->points.rend(),
                        [](const AbsPoint *p) { return p->state == AbsPoint::State::DYNAMIC_POS; }).base();
                    bool trailingStaticPoints = firstStaticAfterLastDynamic != currentGroup->points.begin();

                    Group before, after;

                    static constexpr meter_t MAX_OBSTACLE_LENGTH_AFTER_DYNAMIC_POINT = meter_t(1.0f);

                    if (beginningStaticPoints) {
                        const meter_t distBeforeFirstDynamic = currentGroup->points[0]->absMapPos.distance((*firstDynamic)->absMapPos);
                        if (distBeforeFirstDynamic > MAX_OBSTACLE_LENGTH_AFTER_DYNAMIC_POINT) {
                            // separates starting static points from middle dynamic points
                            // in order to prevent wall from being added to a moving object
                            for (std::vector<AbsPoint*>::iterator it = currentGroup->points.begin(); it != firstDynamic; ) {
                                before.points.push_back(*it);
                                it = currentGroup->points.erase(it);
                                --firstDynamic;
                                --firstStaticAfterLastDynamic;
                            }
                        }
                    }

                    if (trailingStaticPoints) {
                        const meter_t distAfterLastDynamic = currentGroup->points.back()->absMapPos.distance((*(firstStaticAfterLastDynamic - 1))->absMapPos);
                        if (distAfterLastDynamic > MAX_OBSTACLE_LENGTH_AFTER_DYNAMIC_POINT) {
                            // separates trailing static points from middle dynamic points
                            // in order to prevent wall from being added to a moving object
                            for (std::vector<AbsPoint*>::iterator it = firstStaticAfterLastDynamic; it != currentGroup->points.end(); ) {
                                after.points.push_back(*it);
                                it = currentGroup->points.erase(it);
                            }
                        }
                    }

                    Group current = *currentGroup;
                    groups.erase(currentGroup);

                    groups.push_back(before);
                    groups.push_back(current);
                    groups.push_back(after);
                }

                groups.push_back(Group());
                currentGroup = groups.end() - 1;
            }

            if (currentGroup != groups.end()) {
                currentGroup->points.push_back(&p);
            }

            p_prev = &p;
            i = bcr::incr_overflow(i, meas.points.size());

        } while (i != startIdx);


        static constexpr meter_t MIN_OBSTACLE_RADIUS = centimeter_t(2.5);
        static constexpr meter_t MAX_OBSTACLE_RADIUS = centimeter_t(100.0);

        for (std::vector<Group>::iterator it = groups.begin(); it != groups.end(); ) {
            switch(it->points.size()) {
                case 1:
                    it->points[0]->state = AbsPoint::State::STATIC;
                    /* no break */
                case 0:
                    it = groups.erase(it);
                    break;
                default:
                    it->updateIsMoving();
                    if (it->isMoving()) {
                        // for (AbsPoint *p : it->points) {
                        //     it->center += p->absMapPos;
                        // }
                        // it->center /= it->points.size();

                        AbsPoint *p1 = nullptr;
                        AbsPoint *p2 = nullptr;
                        meter_t maxDist(0);

                        for (uint32_t i = 0; i < it->points.size(); ++i) {
                            AbsPoint *pi = it->points[i];
                            for (uint32_t j = i; j < it->points.size(); ++j) {
                                AbsPoint *pj = it->points[j];
                                const meter_t dist = pi->absMapPos.distance(pj->absMapPos);
                                if (dist > maxDist) {
                                    maxDist = dist;
                                    p1 = pi;
                                    p2 = pj;
                                }
                            }
                        }

                        it->center = avg(p1->absMapPos, p2->absMapPos);
                        it->radius = meter_t(0);

                        for (AbsPoint *p : it->points) {
                            const meter_t dist = p->absMapPos.distance(it->center);
                            if (dist > it->radius) {
                                it->radius = dist;
                            }
                        }

                        if (it->radius < MIN_OBSTACLE_RADIUS || it->radius > MAX_OBSTACLE_RADIUS) {
                            it = groups.erase(it);
                            break;
                        }

                    }
                    ++it;
            }
        }
    }

    return groups;
}

void sendObstacles(const std::vector<Group>& groups) {
    visualization_msgs::MarkerArray obstacles;
    environment_builder::DynamicObjectArray dynamicObjects;

    for (const Group& g : groups) {
        if (g.isMoving()) {
            visualization_msgs::Marker speed_marker;
            speed_marker.header.frame_id = "odom";
            speed_marker.header.stamp = ros::Time();
            speed_marker.ns = "bcr";
            speed_marker.id = g.idx;
            speed_marker.type = visualization_msgs::Marker::ARROW;
            speed_marker.action = visualization_msgs::Marker::ADD;
            speed_marker.pose = bcr::ros_convert(bcr::Pose{ g.center, g.speed.getAngle() });
            speed_marker.scale.x = 1 * g.speed.length().get();
            speed_marker.scale.y = 0.1;
            speed_marker.scale.z = 0.1;
            speed_marker.color.a = 1.0;
            speed_marker.color.r = 0.0;
            speed_marker.color.g = 1.0;
            speed_marker.color.b = 0.0;
            speed_marker.lifetime = ros::Duration(0.5);
            obstacles.markers.push_back(speed_marker);

            visualization_msgs::Marker object_marker;
            object_marker.header.frame_id = "odom";
            object_marker.header.stamp = ros::Time();
            object_marker.ns = "bcr";
            object_marker.id = std::numeric_limits<int32_t>::max() - g.idx;
            object_marker.type = visualization_msgs::Marker::SPHERE;
            object_marker.action = visualization_msgs::Marker::ADD;
            object_marker.pose = bcr::ros_convert(bcr::Pose{ g.center, g.speed.getAngle() });
            object_marker.scale.x = g.radius.get();
            object_marker.scale.y = g.radius.get();
            object_marker.scale.z = 1.0;
            object_marker.color.a = 1.0;
            object_marker.color.r = 0.0;
            object_marker.color.g = 0.0;
            object_marker.color.b = 1.0;
            object_marker.lifetime = ros::Duration(0.5);
            obstacles.markers.push_back(object_marker);

            environment_builder::DynamicObject dynObj;
            dynObj.radius = g.radius.get();
            dynObj.pose   = bcr::ros_convert(bcr::Pose{ g.center, g.speed.getAngle() });
            dynObj.twist  = bcr::ros_convert(bcr::Twist{ g.speed, rad_per_sec_t::ZERO() });
            dynamicObjects.objects.push_back(dynObj);
        }
    }

    diffGrid.info.width = diffGrid.info.height = MAP_SIZE / MAP_RES;
    diffGrid.info.resolution = static_cast<meter_t>(MAP_RES).get();

    obstaclesPub->publish(obstacles);
    dynamicObjectsPub->publish(dynamicObjects);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    prevScans.emplace_back(scan);
    LaserMeas& meas = prevScans[prevScans.size() - 1];
    meas.odom = carOdom;
    meas.mapPose = transformPose("odom", "map", carOdom.pose);
    staticScan = *scan;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Vector3 origin(-static_cast<meter_t>(MAP_SIZE).get() / 2.0, -static_cast<meter_t>(MAP_SIZE).get() / 2.0, 0.0);
    transform.setOrigin(origin);
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "diff_grid"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "static_grid"));
    
    diffGrid.header.frame_id = "diff_grid";
    diffGrid.header.stamp = scan->header.stamp;

    diffGrid.info.width = diffGrid.info.height = MAP_SIZE / MAP_RES;
    diffGrid.info.resolution = static_cast<meter_t>(MAP_RES).get();

    diffGrid.info.origin.position.x = 0;
    diffGrid.info.origin.position.y = 0;
    diffGrid.info.origin.position.z = 0;
    tf::quaternionTFToMsg(q, diffGrid.info.origin.orientation);

    diffGrid.info.map_load_time = diffGrid.header.stamp;

    diffGrid.data.resize(diffGrid.info.width * diffGrid.info.height);
    std::fill(diffGrid.data.begin(), diffGrid.data.end(), 0);

    absMap.grid.header.frame_id = "static_grid";
    absMap.grid.header.stamp = scan->header.stamp;

    absMap.grid.info.width = absMap.getWidth();
    absMap.grid.info.height = absMap.getHeight();
    absMap.grid.info.resolution = absMap.getResolution();

    tf::quaternionTFToMsg(q, absMap.grid.info.origin.orientation);

    absMap.grid.info.map_load_time = absMap.grid.header.stamp;

    absMap.updateCenter(carOdom.pose.pos);

    ROS_INFO("------------------------------------------------");

    for(int32_t i = 0; i < meas.count; i++) {
        const radian_t angle = radian_t(scan->angle_min + scan->angle_increment * i);
        const meter_t dist = bcr::min(meter_t(scan->ranges[i]), LIDAR_MAX_DIST);

        // filters out too near objects (false detection)
        if (dist > centimeter_t(10)) {
            const Point2m lidarPoint(dist * bcr::cos(angle), dist * bcr::sin(angle));

#if SINGLE_LIDAR
            const Point2m absMapPoint = transformPoint("scanner", "map", lidarPoint);
#else
            const Point2m absMapPoint = transformPoint("front_scanner", "map", lidarPoint);//.average(transformPoint("rear_scanner", "map", lidarPoint));
#endif

            //meas.points.push_back(absPoint);
            meas.points.push_back({ i, dist, absMapPoint, AbsPoint::State::STATIC });
            //absMap.set(absMapPoint, AbsoluteMap::CellState::OCCUPIED);
        }
    }

    if (prevScans.size() >= NUM_PREV_COMPARE + 1) { // checks if enough samples have been collected

        std::vector<AbsPoint*> diffPoints;

        // checks last NUM_PREV_COMPARE measurements, if point is not present in any of them, marks it as dynamic point

        for (uint32_t i = 0; i < meas.points.size(); ++i) {
            AbsPoint& p = meas.points[i];
            const meter_t cur_dist = meter_t(scan->ranges[p.idx]);
            const meter_t eps = cur_dist * scan->angle_increment * 1.5;

            for (uint32_t s = 1; s <= NUM_PREV_COMPARE; ++s) {
                const LaserMeas& prev = prevScans[bcr::sub_underflow(prevScans.size() - 1, s, prevScans.size())];

                // checks if correspondent point can be found in the previous measurement
                // if yes, it means point is a static point
                // if no, then dynamic


                // if point is not found in one of the previous measurements, it may be a dynamic point
                // but first checks if current distance is smaller than the previous, otherwise it is not a moving object that we have detected,
                // but the wall behind an object that is moving away
                if (std::find_if(prev.points.begin(), prev.points.end(), [&p, eps](const AbsPoint& prevPoint) { return p.absMapPos.distance(prevPoint.absMapPos) < eps; }) == prev.points.end()) {
                    const meter_t prev_dist = prev.getDistanceInDirection(p.absMapPos);
                    //const radian_t prev_angle = prev.odom.pose.pos.getAngle(p.absMapPos) - prev.odom.pose.angle;
                    //const meter_t prev_dist = prev.getDistance(prev_angle);
                    //const Point2m prevAbsPoint = rayDistToAbsPoint(prev, prev_angle, prev_dist);

                    if (cur_dist < prev_dist + centimeter_t(50)) {
                        p.state = AbsPoint::State::DYNAMIC_POS;
                        diffPoints.push_back(&p);
                        //ROS_INFO("odom: %f, %f, angle: %f", meas.odom.pose.pos.X.get(), meas.odom.pose.pos.Y.get(), meas.odom.pose.angle.get());
                        //ROS_INFO("prev odom: %f, %f, angle: %f", prev.odom.pose.pos.X.get(), prev.odom.pose.pos.Y.get(), prev.odom.pose.angle.get());
                        //ROS_INFO("dist: %f, prev: %f", cur_dist.get(), prev_dist.get());
                        //diffGrid.data[p.mapGridPos.Y * diffGrid.info.width + p.mapGridPos.X] = 100;
                        break;
                    }
                }
            }
        }
        

        // finds negative dynamic points (that existed in the previous measurement but not in the current one)

        std::vector<radian_t> negDiffAngles;
        const LaserMeas& prevMeas = prevScans[bcr::sub_underflow(prevScans.size() - 1, NUM_PREV_COMPARE, prevScans.size())];

        for (uint32_t i = 0; i < prevMeas.points.size(); ++i) {
            const AbsPoint& p = prevMeas.points[i];

             for (uint32_t s = 1; s <= NUM_PREV_COMPARE - 1; ++s) {
                const LaserMeas& m = prevScans[bcr::sub_underflow(prevScans.size() - 1, s, prevScans.size())];

                // checks if correspondent point can be found in the other previous measurements or the current measurement
                // if yes, it means point is a static point
                // if no, then dynamic

                const meter_t eps = p.absMapPos.length() * scan->angle_increment / 4;

                // if point is not found in one of the previous measurements or the current measurement, it may be a negative dynamic point
                // but first checks if distance is smaller than the other distances, otherwise it is not a moving object that we have detected,
                // but the wall behind an object that is moving away
                if (std::find_if(m.points.begin(), m.points.end(), [&p, eps](const AbsPoint& prevPoint) { return p.absMapPos.distance(prevPoint.absMapPos) < eps; }) == m.points.end()) {
                    const meter_t dist = meter_t(prevMeas.scan->ranges[p.idx]);
                    const meter_t other_dist = m.getDistanceInDirection(p.absMapPos);

                    //ROS_INFO("possible diff point: %d: (%d, %d)", p.idx, p.mapGridPos.X, p.mapGridPos.Y);

                    if (other_dist < dist + centimeter_t(20)) {
                        const radian_t current_angle = meas.odom.pose.pos.getAngle(p.absMapPos);
                        negDiffAngles.push_back(current_angle);
                        //ROS_INFO("negDiffAngle: %f deg", static_cast<degree_t>(current_angle).get());
                        //diffGrid.data[p.mapGridPos.Y * diffGrid.info.width + p.mapGridPos.X] = 100;
                    }

                    break;
                }
            }
        }

        // removes noise - only keeps dynamic points that have left and right dynamic neighbours in the scan's ranges

        //std::vector<AbsPoint*> filteredDiffPoints;
        //filteredDiffPoints.reserve(diffPoints.size());

        for (uint32_t i = 0; i < diffPoints.size(); ++i) {
            AbsPoint *p = diffPoints[i];

            const uint32_t prevIdx = bcr::decr_underflow(i, diffPoints.size());
            const uint32_t nextIdx = bcr::incr_overflow(i, diffPoints.size());

            const uint32_t prevRayIdx = bcr::decr_underflow(p->idx, meas.count);
            const uint32_t nextRayIdx = bcr::incr_overflow(p->idx, meas.count);

            //ROS_INFO("idx: %d, %d, %d (%d)", i, prevIdx, nextIdx, diffPoints.size());
            //ROS_INFO("ray: %d, %d, %d (%d)", p.idx, prevRayIdx, nextRayIdx, count);

            // checks if the stored dynamic points before and after the given dynamic point are from the neighbouring scans
            // if yes, then current dynamic point is in a batch of dynamic points - we keep it
            // if no, then current dynamic point is probably just noise - we drop it

            if (diffPoints[prevIdx]->idx != prevRayIdx && diffPoints[nextIdx]->idx != nextRayIdx) {
                //ROS_INFO("not diff point: %d: (%d, %d)", p.idx, p.mapGridPos.X, p.mapGridPos.Y);
                //diffGrid.data[p->mapGridPos.Y * diffGrid.info.width + p->mapGridPos.X] = 0;
                p->state = AbsPoint::State::STATIC;
            } else {
                //ROS_INFO("diff point: %d: (%d, %d)", p.idx, p.mapGridPos.X, p.mapGridPos.Y);
                //filteredDiffPoints.push_back(p);
            }
        }

        std::vector<Group> groups = makeGroups(meas, radian_t(scan->angle_increment), negDiffAngles);
        std::vector<Group> filteredGroups;
        filteredGroups.reserve(groups.size());

        // finds moving objects
        if (prevGroupsList.size() >= NUM_SPEED_FILTER + 1) {
            const std::vector<Group>& prevGroups = prevGroupsList[prevGroupsList.size() - 1];
            const millisecond_t prev_current_dt  = meas.time - prevScans[bcr::sub_underflow(prevScans.size() - 1, 1, prevScans.size())].time;
            const millisecond_t speedFilter_dt   = meas.time - prevScans[bcr::sub_underflow(prevScans.size() - 1, NUM_SPEED_FILTER, prevScans.size())].time;

            for (const Group& prev : prevGroups) {
                if (prev.isMoving()) {

                    // checks if group exised in the previous measurement
                    // if yes, inherits id
                    meter_t minDist = centimeter_t(80);
                    std::vector<Group>::iterator nearest = groups.end();

                    for (std::vector<Group>::iterator it = groups.begin(); it != groups.end(); ++it) {
                        if (it->isMoving()) {
                            const meter_t dist = (prev.center + prev.speed * prev_current_dt).distance(it->center);
                            if (dist < minDist) {
                                minDist = dist;
                                nearest = it;
                            }
                        }
                    }

                    if (nearest != groups.end()) {
                        nearest->idx = prev.idx;
                        nearest->existingCntr = prev.existingCntr + 1;
                        nearest->radius = avg(nearest->radius, prev.radius);
                    }
                }
            }

            for (Group& g : groups) {
                if (g.isMoving()) {
                    // new group -> new id
                    if (g.idx == Group::INVALID_IDX) {
                        g.idx = ++maxGroupIdx;
                        g.existingCntr = 1;
                    }
                }
            }

            static constexpr uint32_t MAX_GROUP_FORCE_APPEAR_COUNT = 5;

            for (const Group& prev : prevGroups) {
                // if previous group is not found in the current measurement, makes it appear for awhile
                if (prev.forcedCntr < MAX_GROUP_FORCE_APPEAR_COUNT && prev.isMoving()) {

                    if (std::find_if(groups.begin(), groups.end(), [&prev] (const Group& g) { return prev.idx == g.idx; }) == groups.end()) {
                        groups.push_back(Group::createFromPrev(prev, prev_current_dt));
                    }
                }
            }

            // joins contiguous groups
            for (std::vector<Group>::iterator g1 = groups.begin(); g1 != groups.end(); ++g1) {
                for (std::vector<Group>::iterator g2 = g1 + 1; g2 != groups.end();) {
                    if (g1->center.distance(g2->center) < centimeter_t(25) && (g1->existingCntr == 1 || g2->existingCntr == 1)) {
                        g1->merge(*g2);
                        g2 = groups.erase(g2);
                    } else {
                        ++g2;
                    }
                }
            }

            for (Group& g : groups) {
                if (g.isMoving() && !g.forcedCntr) {

                    // calculates group speed
                    const std::vector<Group>& prevGroups = prevGroupsList[bcr::sub_underflow(prevGroupsList.size(), NUM_SPEED_FILTER, prevGroupsList.size())];
                    std::vector<Group>::const_iterator it = std::find_if(prevGroups.begin(), prevGroups.end(), [&g] (const Group& pg) { return pg.idx == g.idx; });

                    if (it != prevGroups.end()) {
                        g.speed = { (g.center.X - it->center.X) / speedFilter_dt, (g.center.Y - it->center.Y) / speedFilter_dt };
                        //ROS_INFO("Group #%d: (%f - %f) m / %f ms = %f m/s", g.idx, g.center.X.get(), g.center.Y.get(), speedFilter_dt.get(), g.speed.length().get());
                        //ROS_INFO("sp: %f", g.speed.length().get());
                    } else {
                        g.speed = { m_per_sec_t::ZERO(), m_per_sec_t::ZERO() };
                    }
                }
            }

            for (Group& g : groups) {
                if (g.isMoving()) {
                    const m_per_sec_t speed = g.speed.length();
                    if (bcr::isinf(speed)) {
                        g.speed = { m_per_sec_t::ZERO(), m_per_sec_t::ZERO() };
                    }

                    if (bcr::abs(speed) > m_per_sec_t(0.05f) &&
                        (carOdom.pose.pos.distance(g.center) > meter_t(1.5) || g.existingCntr > 1)) {

                        filteredGroups.push_back(g);
                        const std::string forced = g.forcedCntr > 0 ? " (forced: " + std::to_string(g.forcedCntr) + ")" : "";
                        ROS_INFO("Group #%d: %f, %f%s - speed: { %f, %f } (%f) m/s", g.idx, g.center.X.get(), g.center.Y.get(), forced.c_str(), g.speed.X.get(), g.speed.Y.get(), g.speed.length().get());
                    }
                }
            }

        } else {
            for (Group& g : groups) {
                if (g.isMoving()) {
                    g.idx = ++maxGroupIdx;
                }
            }

            filteredGroups = groups;
        }

        prevGroupsList.append(groups);

        for (Group& g : filteredGroups) {
            for (AbsPoint *p : g.points) {
                p->state = AbsPoint::State::DYNAMIC_POS;
            }
        }

        // removes dynamic points from the static scan and adds static points to the absolute map

        for (const AbsPoint& p : meas.points) {
            if (p.state == AbsPoint::State::DYNAMIC_POS) {
                staticScan.ranges[p.idx] = std::numeric_limits<float32_t>::infinity();
            } else if (p.state == AbsPoint::State::STATIC) {
                if (p.dist == LIDAR_MAX_DIST) {
                    absMap.setRay(meas.odom.pose.pos, carOdom.pose.pos.getAngle(p.absMapPos));
                } else {
                    absMap.setRay(meas.odom.pose.pos, p.absMapPos);
                }
            }
        }

        sendObstacles(filteredGroups);
        diffGridPub->publish(diffGrid);
        staticGridPub->publish(absMap.grid);
        staticScanPub->publish(staticScan);
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    carOdom_ros = *odom;
    carOdom = bcr::ros_convert(*odom);
}

} // namespace

int main(int argc, char **argv)
{
    static const std::string NODE_NAME = "environment_builder__decomposer";
    ros::init(argc, argv, NODE_NAME);
    node.reset(new DecomposerNode(NODE_NAME));
    ros::Subscriber scanSub = node->subscribe<sensor_msgs::LaserScan>("lidar_scan", 1, scanCallback);
    ros::Subscriber odomSub = node->subscribe<nav_msgs::Odometry>("odom", 1, odomCallback);

    ros::Publisher diffGridPublisher = node->advertise<nav_msgs::OccupancyGrid>("diff_grid", 10);
    diffGridPub = &diffGridPublisher;

    ros::Publisher staticGridPublisher = node->advertise<nav_msgs::OccupancyGrid>("static_grid", 10);
    staticGridPub = &staticGridPublisher;

    ros::Publisher staticScanPublisher = node->advertise<sensor_msgs::LaserScan>("static_scan", 10);
    staticScanPub = &staticScanPublisher;

    ros::Publisher dynamicObjectsPublisher = node->advertise<environment_builder::DynamicObjectArray>("dynamic_objects", 10);
    dynamicObjectsPub = &dynamicObjectsPublisher;

    ros::Publisher obstaclesPublisher = node->advertise<visualization_msgs::MarkerArray>("obstacles", 0);
    obstaclesPub = &obstaclesPublisher;

    absMap.initialize(MAP_SIZE, MAP_RES);

    staticScan.ranges.reserve(MAX_SCAN_SAMPLES);

    dynamicObjects.reserve(50);

    ros::spin();

    return 0;
}
