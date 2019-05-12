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
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <algorithm>

//#define SCAN_TOPIC "scan"
#define SCAN_TOPIC "lidar_scan"
#define SCAN_TF_FRAME "scanner"

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

// TODO use tf

nav_msgs::Odometry carOdom_ros;
Odometry carOdom;

static constexpr uint32_t MAX_SCAN_SAMPLES = 400;

AbsoluteMap absMap;

struct AbsPoint {
    enum class State {
        UNKNOWN,
        STATIC,
        DYNAMIC_NEG,    // negative-dynamic point (distant point visible after closer object has passed)
        DYNAMIC_POS     // positive-dynamic point (object passing in front of a distant point)
    };

    int32_t idx;
    Point2m absOdomPos;
    Point2m absMapPos;
    Point2i mapGridPos;
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
};

struct Group {
    static constexpr int32_t INVALID_IDX = -1;
    int32_t idx;
    uint32_t forcedCntr;    // indicates if group has not been detected in the current measurement, but has been forced to exist (filtering)
    Point2m center;
    meter_t radius;
    std::vector<AbsPoint*> points;
    Vec2<m_per_sec_t> speed;

    Group()
        : idx(INVALID_IDX)
        , forcedCntr(0) {}

    static Group createFromPrev(const Group& prev, millisecond_t dt) {
        Group result;
        result.idx = prev.idx;
        result.center = prev.center + prev.speed * dt;
        result.radius = prev.radius;
        result.forcedCntr = prev.forcedCntr + 1;
        result.speed = prev.speed;
        return result;
    }

    bool isMoving() const {
        return this->forcedCntr > 0 ||
            std::count_if(this->points.begin(), this->points.end(), [](const AbsPoint *p) { return p->state == AbsPoint::State::DYNAMIC_POS; }) >= 2;
    }
};

static constexpr uint32_t NUM_PREV_COMPARE = 5;
static constexpr uint32_t NUM_SPEED_FILTER = 5;

ring_buffer<LaserMeas, NUM_PREV_COMPARE + 1> prevScans;
ring_buffer<std::vector<Group>, NUM_PREV_COMPARE + 1> prevGroupsList;

sensor_msgs::LaserScan staticScan;
nav_msgs::OccupancyGrid diffGrid;

std::vector<DynamicObject> dynamicObjects;

ros::Publisher *diffGridPub = nullptr;
ros::Publisher *staticGridPub = nullptr;
ros::Publisher *staticScanPub = nullptr;
ros::Publisher *obstaclesPub = nullptr;

int32_t maxGroupIdx = Group::INVALID_IDX;

Pose transformPose(const std::string& from_frame, const std::string& to_frame, const Pose& p) {

    geometry_msgs::PoseStamped from_pose;
    geometry_msgs::PoseStamped to_pose;

    from_pose.header.frame_id = from_frame;
    from_pose.header.stamp = ros::Time();
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

Point2m transformPoint(const std::string& from_frame, const std::string& to_frame, const Point2m& p) {

    geometry_msgs::PointStamped from_point;
    geometry_msgs::PointStamped to_point;

    from_point.header.frame_id = from_frame;
    from_point.header.stamp = ros::Time();
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

meter_t getDistInDirection(const LaserMeas& meas, const Point2m& p) {
    const radian_t angle = meas.odom.pose.pos.getAngle(p) - meas.odom.pose.angle;
    return meas.getDistance(angle);
}

std::vector<Group> makeGroups(LaserMeas& meas, radian_t ray_angle_incr, const std::vector<radian_t>& negDiffAngles) {
    std::vector<Group> groups;

    if (meas.points.size() >= 2) {

        std::vector<Group>::iterator currentGroup = groups.end();

        const AbsPoint *p_prev = &meas.points[0];
        int32_t startIdx = 1;
        int32_t i = 1;

        do {
            AbsPoint& p = meas.points[i];
            const meter_t eps = bcr::clamp(meter_t(p.absOdomPos.length() * ray_angle_incr * 4), meter_t(0.2f), meter_t(0.4f));

            // if point is far from its neighbour, we start a new group (new object)
            if (p.absOdomPos.distance(p_prev->absOdomPos) > eps) {
                // saves index of the first point thas has already been added to a group
                if (currentGroup == groups.end()) {
                    startIdx = i;
                } else {
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
                        const meter_t distBeforeFirstDynamic = currentGroup->points[0]->absOdomPos.distance((*firstDynamic)->absOdomPos);
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
                        const meter_t distAfterLastDynamic = currentGroup->points.back()->absOdomPos.distance((*(firstStaticAfterLastDynamic - 1))->absOdomPos);
                        if (distAfterLastDynamic > MAX_OBSTACLE_LENGTH_AFTER_DYNAMIC_POINT) {
                            // separates trailing static points from middle dynamic points
                            // in order to prevent wall from being added to a moving object
                            for (std::vector<AbsPoint*>::iterator it = firstStaticAfterLastDynamic; it != currentGroup->points.end(); ) {
                                after.points.push_back(*it);
                                it = currentGroup->points.erase(it);
                            }
                        }
                    }

                    groups.push_back(before);   // TODO order should be before, current, after (now: current, before, after)
                    groups.push_back(after);
                }

                groups.push_back(Group());
                currentGroup = groups.end() - 1;
            } else {
                // if (p.state != AbsPoint::State::DYNAMIC_POS) {
                //     const radian_t prev_angle = meas.odom.pose.pos.getAngle(p_prev->absOdomPos);
                //     const radian_t angle = meas.odom.pose.pos.getAngle(p.absOdomPos);

                //     bool found = false;
                //     for (radian_t ang : negDiffAngles) {
                //         if (bcr::isBtw(ang, angle, prev_angle)) {
                //             found = true;
                //             break;                        
                //         }
                //     }

                //     if (found) {
                //         ROS_INFO("forced new group");
                //         groups.push_back(Group());
                //         currentGroup = groups.end() - 1;
                //     }
                // }
            }

            if (currentGroup != groups.end()) {
                currentGroup->points.push_back(&p);
            }

            p_prev = &p;
            i = bcr::incr_overflow(i, meas.points.size());

        } while (i != startIdx);

        // TODO cut off wall from objects near it

        for (std::vector<Group>::iterator it = groups.begin(); it != groups.end(); ) {
            switch(it->points.size()) {
                case 1:
                    it->points[0]->state = AbsPoint::State::STATIC;
                    /* no break */
                case 0:
                    it = groups.erase(it);
                    break;
                default:
                    if (it->isMoving()) {
                        for (AbsPoint *p : it->points) {
                            p->state = AbsPoint::State::DYNAMIC_POS;
                            it->center += p->absOdomPos;
                        }
                        it->center /= it->points.size();

                        for (AbsPoint *p : it->points) {
                            const meter_t dist = p->absOdomPos.distance(it->center);
                            if (dist > it->radius) {
                                it->radius = dist;
                            }
                        }
                    } else {
                        for (AbsPoint *p : it->points) {
                            p->state = AbsPoint::State::STATIC;
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

    for (const Group& g : groups) {
        if (g.isMoving()) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = ros::Time();
            marker.ns = "bcr";
            marker.id = g.idx;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = bcr::ros_convert(bcr::Pose{ g.center, Point2mps(speed_t::ZERO(), speed_t::ZERO()).getAngle(g.speed) });
            marker.scale.x = 1 * g.speed.length().get();
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.lifetime = ros::Duration(0.5);

            obstacles.markers.push_back(marker);
        }
    }

    diffGrid.info.width = diffGrid.info.height = MAP_SIZE / MAP_RES;
    diffGrid.info.resolution = static_cast<meter_t>(MAP_RES).get();

    obstaclesPub->publish(obstacles);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    ROS_INFO("New scan");
    prevScans.emplace_back(scan);
    LaserMeas& meas = prevScans[prevScans.size() - 1];
    meas.odom = carOdom;
    meas.mapPose = transformPose("odom", "map", carOdom.pose);
    staticScan = *scan;

    const double theta = 0.0;

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Vector3 origin(-static_cast<meter_t>(MAP_SIZE).get() / 2.0, -static_cast<meter_t>(MAP_SIZE).get() / 2.0, 0.0);
    transform.setOrigin(origin);
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, theta);
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

    absMap.grid.info.width = absMap.grid.info.height = MAP_SIZE / MAP_RES;
    absMap.grid.info.resolution = static_cast<meter_t>(MAP_RES).get();

    absMap.grid.info.origin.position.x = 0;
    absMap.grid.info.origin.position.y = 0;
    absMap.grid.info.origin.position.z = 0;
    tf::quaternionTFToMsg(q, absMap.grid.info.origin.orientation);

    absMap.grid.info.map_load_time = absMap.grid.header.stamp;

    ROS_INFO("------------------------------------------------");

    for(int32_t i = 0; i < meas.count; i++) {
        const radian_t angle = radian_t(scan->angle_min + scan->angle_increment * i);
        const meter_t dist = bcr::min(meter_t(scan->ranges[i]), LIDAR_MAX_DIST);

        // filters out too near objects (false detection)
        if (dist > centimeter_t(10)) {
            const Point2m lidarPoint(dist * bcr::cos(angle), dist * bcr::sin(angle));
            const Point2m absOdomPoint = transformPoint(SCAN_TF_FRAME, "odom", lidarPoint);
            const Point2m absMapPoint = transformPoint("odom", "map", absOdomPoint);
            //meas.points.push_back(absPoint);
            const Point2i absMapIndexes = absMap.getNearestIndexes(absMapPoint);
            meas.points.push_back({ i, absOdomPoint, absMapPoint, absMapIndexes, AbsPoint::State::STATIC });
            //absMap.set(absMapPoint, AbsoluteMap::CellState::OCCUPIED);
        }
    }

    if (prevScans.size() >= NUM_PREV_COMPARE + 1) { // checks if enough samples have been collected

        std::vector<AbsPoint*> diffPoints;

        // checks last NUM_PREV_COMPARE measurements, if point is not present in any of them, marks it as dynamic point

        for (uint32_t i = 0; i < meas.points.size(); ++i) {
            AbsPoint& p = meas.points[i];
            const meter_t cur_dist = meter_t(scan->ranges[p.idx]);
            const meter_t eps = cur_dist * scan->angle_increment * 2;

            for (uint32_t s = 1; s <= NUM_PREV_COMPARE; ++s) {
                const LaserMeas& prev = prevScans[bcr::sub_underflow(prevScans.size() - 1, s, prevScans.size())];

                // checks if correspondent point can be found in the previous measurement
                // if yes, it means point is a static point
                // if no, then dynamic


                // if point is not found in one of the previous measurements, it may be a dynamic point
                // but first checks if current distance is smaller than the previous, otherwise it is not a moving object that we have detected,
                // but the wall behind an object that is moving away
                if (std::find_if(prev.points.begin(), prev.points.end(), [&p, eps](const AbsPoint& prevPoint) { return p.absOdomPos.distance(prevPoint.absOdomPos) < eps; }) == prev.points.end()) {
                    const meter_t prev_dist = getDistInDirection(prev, p.absOdomPos);
                    //const radian_t prev_angle = prev.odom.pose.pos.getAngle(p.absOdomPos) - prev.odom.pose.angle;
                    //const meter_t prev_dist = prev.getDistance(prev_angle);
                    //const Point2m prevAbsPoint = rayDistToAbsPoint(prev, prev_angle, prev_dist);

                    if (cur_dist < prev_dist + centimeter_t(50)) {
                        p.state = AbsPoint::State::DYNAMIC_POS;
                        diffPoints.push_back(&p);
                        //ROS_INFO("odom: %f, %f, angle: %f", meas.odom.pose.pos.X.get(), meas.odom.pose.pos.Y.get(), meas.odom.pose.angle.get());
                        //ROS_INFO("prev odom: %f, %f, angle: %f", prev.odom.pose.pos.X.get(), prev.odom.pose.pos.Y.get(), prev.odom.pose.angle.get());
                        //ROS_INFO("diff point: (%f, %f) - dist: %f, prev: (%f, %f) - dist: %f", p.absOdomPos.X.get(), p.absOdomPos.Y.get(), cur_dist.get(), prevAbsPoint.X.get(), prevAbsPoint.Y.get(), prev_dist.get());
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

                const meter_t eps = p.absOdomPos.length() * scan->angle_increment / 4;

                // if point is not found in one of the previous measurements or the current measurement, it may be a negative dynamic point
                // but first checks if distance is smaller than the other distances, otherwise it is not a moving object that we have detected,
                // but the wall behind an object that is moving away
                if (std::find_if(m.points.begin(), m.points.end(), [&p, eps](const AbsPoint& prevPoint) { return p.absOdomPos.distance(prevPoint.absOdomPos) < eps; }) == m.points.end()) {
                    const meter_t dist = meter_t(prevMeas.scan->ranges[p.idx]);
                    const meter_t other_dist = getDistInDirection(m, p.absOdomPos);

                    //ROS_INFO("possible diff point: %d: (%d, %d)", p.idx, p.mapGridPos.X, p.mapGridPos.Y);

                    if (other_dist < dist + centimeter_t(20)) {
                        const radian_t current_angle = meas.odom.pose.pos.getAngle(p.absOdomPos);
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

            if (diffPoints[prevIdx]->idx != prevRayIdx || diffPoints[nextIdx]->idx != nextRayIdx) {
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

            for (Group& g : groups) {
                if (g.isMoving()) {

                    meter_t minDist = centimeter_t(40);
                    for (const Group& prev : prevGroups) {
                        if (prev.isMoving()) {
                            const meter_t dist = (prev.center + prev.speed * prev_current_dt).distance(g.center);
                            //ROS_INFO("dist: %f", dist.get());
                            if (dist < minDist) {
                                minDist = dist;
                                g.idx = prev.idx;
                            }
                        }
                    }

                    if (g.idx == Group::INVALID_IDX) {
                        g.idx = ++maxGroupIdx;
                    }
                }
            }

            static constexpr uint32_t MAX_GROUP_FORCE_APPEAR_COUNT = 3;

            for (const Group& prev : prevGroups) {
                if (prev.forcedCntr < MAX_GROUP_FORCE_APPEAR_COUNT && prev.isMoving()) {
                    bool found = false;
                    for (const Group& g : groups) {
                        if (prev.idx == g.idx) {
                            found = true;
                            break;
                        }
                    }

                    if (!found) {
                        groups.push_back(Group::createFromPrev(prev, prev_current_dt));
                    }
                }
            }

            for (Group& g : groups) {
                if (g.isMoving()) {

                    const std::vector<Group>& prevGroups = prevGroupsList[bcr::sub_underflow(prevGroupsList.size(), NUM_SPEED_FILTER, prevGroupsList.size())];
                    std::vector<Group>::const_iterator it;
                    for (it = prevGroups.begin(); it != prevGroups.end(); ++it) {
                        if (it->idx == g.idx) {
                            break;
                        }
                    }

                    if (it != prevGroups.end()) {
                        g.speed = { (g.center.X - it->center.X) / speedFilter_dt, (g.center.Y - it->center.Y) / speedFilter_dt };
                        ROS_INFO("Group #%d: (%f - %f) m / %f ms = %f m/s", g.idx, g.center.X.get(), g.center.Y.get(), speedFilter_dt.get(), g.speed.length().get());
                        //ROS_INFO("sp: %f", g.speed.length().get());
                    } else {
                        g.speed = { speed_t::ZERO(), speed_t::ZERO() };
                    }
                }
            }

            for (Group& g : groups) {
                if (g.isMoving()) {
                    const m_per_sec_t speed = g.speed.length();
                    if (bcr::abs(speed) > m_per_sec_t(0.05f) && !bcr::isinf(speed)) {
                        filteredGroups.push_back(g);
                        const std::string forced = g.forcedCntr > 0 ? " (forced: " + std::to_string(g.forcedCntr) + ")" : "";
                        //ROS_INFO("Group #%d: %f, %f%s - speed: { %f, %f } (%f) m/s", g.idx, g.center.X.get(), g.center.Y.get(), forced.c_str(), g.speed.X.get(), g.speed.Y.get(), g.speed.length().get());
                        for (const AbsPoint *p : g.points) {
                            //ROS_INFO("group point: %d: (%d, %d)", p->idx, p->mapGridPos.X, p->mapGridPos.Y);
                            diffGrid.data[p->mapGridPos.Y * diffGrid.info.width + p->mapGridPos.X] = 100;
                        }
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

        // removes dynamic points from the static scan and adds static points to the absolute map

        for (const AbsPoint& p : meas.points) {
            if (p.state == AbsPoint::State::DYNAMIC_POS) {
                staticScan.ranges[p.idx] = std::numeric_limits<float32_t>::infinity();
                //absMap.setRay(meas.odom.pose.pos, meas.odom.pose.pos.getAngle(p.absMapPos));
            } else if (p.state == AbsPoint::State::STATIC) {
                absMap.setRay(meas.odom.pose.pos, p.absMapPos);
            }
        }

        sendObstacles(filteredGroups);
        diffGridPub->publish(diffGrid);
        staticGridPub->publish(absMap.grid);
        staticScanPub->publish(staticScan);

        //KMeans massCenters(meas.points);
        //uint32_t numObstacles = 5;  // TODO
        //const std::vector<KMeans::Group> groups = massCenters.run(numObstacles);
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {
    carOdom_ros = *odom;
    carOdom = bcr::ros_convert(*odom);
    //ROS_INFO("Car pos: [%f, %f] m, orientation: %f deg", static_cast<meter_t>(carOdom.pose.pos.X).get(), static_cast<meter_t>(carOdom.pose.pos.Y).get(), static_cast<radian_t>(carOdom.pose.angle).get());
}

} // namespace

int main(int argc, char **argv)
{
    static const std::string NODE_NAME = "environment_builder__decomposer";
    ros::init(argc, argv, NODE_NAME);
    node.reset(new DecomposerNode(NODE_NAME));
    ros::Subscriber scanSub = node->subscribe<sensor_msgs::LaserScan>(SCAN_TOPIC, 1, scanCallback);
    ros::Subscriber odomSub = node->subscribe<nav_msgs::Odometry>("odom", 1, odomCallback);

    ros::Publisher diffGridPublisher = node->advertise<nav_msgs::OccupancyGrid>("diff_grid", 10);
    diffGridPub = &diffGridPublisher;

    ros::Publisher staticGridPublisher = node->advertise<nav_msgs::OccupancyGrid>("static_grid", 10);
    staticGridPub = &staticGridPublisher;

    ros::Publisher staticScanPublisher = node->advertise<sensor_msgs::LaserScan>("static_scan", 10);
    staticScanPub = &staticScanPublisher;

    ros::Publisher obstaclesPublisher = node->advertise<visualization_msgs::MarkerArray>("obstacles", 0);
    obstaclesPub = &obstaclesPublisher;

    absMap.initialize(MAP_SIZE, MAP_RES);

    staticScan.ranges.reserve(MAX_SCAN_SAMPLES);

    dynamicObjects.reserve(50);

    ros::spin();

    return 0;
}