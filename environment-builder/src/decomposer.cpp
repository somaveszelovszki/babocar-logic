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

//#define SCAN_TOPIC "scan"
#define SCAN_TOPIC "lidar_scan"

using namespace bcr;

namespace {

constexpr distance_t MAP_SIZE = meter_t(20);
constexpr distance_t MAP_RES = centimeter_t(5);

class DecomposerNode : public RosNode {
public:
    using RosNode::RosNode;

    tf::TransformListener lidarTransformListener;
};

std::unique_ptr<DecomposerNode> node = nullptr;

// TODO use tf

Odometry carOdom;
Pose lidarPose;

Point2m rayDistToAbsPoint(radian_t angle, meter_t dist) {

    geometry_msgs::PointStamped laser_point;
    geometry_msgs::PointStamped base_point;

    laser_point.header.frame_id = SCAN_TOPIC;
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

struct AbsPoint {
    enum class State {
        UNKNOWN,
        STATIC,
        DYNAMIC
    };

    uint32_t idx;
    Point2m absPos;
    Point2i gridPos;
    State state;
};

struct LaserMeas {
    sensor_msgs::LaserScan::ConstPtr scan;
    int32_t count;
    Odometry odom;
    std::vector<AbsPoint> points;
    
    LaserMeas(const sensor_msgs::LaserScan::ConstPtr& scan)
        : scan(scan) 
        , count((scan->angle_max - scan->angle_min) / scan->angle_increment) {
        this->points.reserve(count);
    }

    meter_t getDistance(radian_t angle) const {
        meter_t dist;

        if (angle.get() < this->scan->angle_min || angle.get() > this->scan->angle_max) {
            dist = distance_t::ZERO();
        } else {
            int32_t idx = bcr::round((angle.get() - this->scan->angle_min) / this->scan->angle_increment);
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
    Point2m center;
    std::vector<AbsPoint*> points;

    bool isMoving() const {
        return std::count_if(this->points.begin(), this->points.end(), [](const AbsPoint *p) { return p->state == AbsPoint::State::DYNAMIC; }) >= 2;
    }
};

meter_t getDistInDirection(const LaserMeas& meas, const Point2m& p) {
    const radian_t angle = meas.odom.pose.pos.getAngle(p);
    return meas.getDistance(angle);
}

ring_buffer<LaserMeas, 5> prevScans;

sensor_msgs::LaserScan staticScan;
nav_msgs::OccupancyGrid diffGrid;

std::vector<DynamicObject> dynamicObjects;

ros::Publisher *diffGridPub = nullptr;

bool isInSameObject(const AbsPoint& p, const AbsPoint& prev, const LaserMeas& meas, radian_t ray_angle_incr) {
    bool connectable = true;
    const meter_t eps = bcr::clamp(meter_t(p.absPos.length() * ray_angle_incr * 2), meter_t(0.1f), meter_t(0.4f));

    if (p.absPos.distance(prev.absPos) > eps) {
        const AbsPoint *p_prev = &prev;
        for (int32_t i = bcr::incr_overflow(p_prev->idx, meas.points.size()); i != p.idx; i = bcr::incr_overflow(i, meas.points.size())) {
            const AbsPoint& cur = meas.points[i];

            if (cur.absPos.distance(p_prev->absPos) > eps) {
                connectable = false;
                break;
            }
        }
    }

    return connectable;
}

std::vector<Group> makeGroups(const std::vector<AbsPoint*>& diffPoints, LaserMeas& meas, radian_t ray_angle_incr) {
    std::vector<Group> groups;

    if (diffPoints.size() > 0 && meas.points.size() >= 2) {

        Group *currentGroup = nullptr;  // TODO use iterator
        const AbsPoint *p_prev = &meas.points[0];
        int32_t startIdx = 1;
        int32_t i = 1;

        do {
            AbsPoint& p = meas.points[i];
            const meter_t eps = bcr::clamp(meter_t(p.absPos.length() * ray_angle_incr * 4), meter_t(0.2f), meter_t(0.4f));

            // if point is far from its neighbour, we start a new group (new object)
            if (p.absPos.distance(p_prev->absPos) > eps) {
                // saves index of the first point thas has already been added to a group
                if (!currentGroup) {
                    startIdx = i;
                } else {
                    // std::vector<AbsPoint*>::iterator firstDynamic = std::find_if(
                    //     currentGroup->points.begin(), currentGroup->points.end(),
                    //     [](const AbsPoint *p) { return p->state == AbsPoint::State::DYNAMIC; });

                    // std::vector<AbsPoint*>::iterator firstStaticAfterLastDynamic = std::find_if(
                    //     currentGroup->points.rbegin(), currentGroup->points.rend(),
                    //     [](const AbsPoint *p) { return p->state == AbsPoint::State::DYNAMIC; }).base();
                    // bool trailingStaticPoints = firstStaticAfterLastDynamic != currentGroup->points.begin();

                    // // separates starting static points from middle dynamic points
                    // // in order to prevent wall from being added to a moving object
                    // Group before, after;
                    // for (std::vector<AbsPoint*>::iterator it = currentGroup->points.begin(); it != firstDynamic; ) {
                    //     before.points.push_back(*it);
                    //     it = currentGroup->points.erase(it);
                    //     --firstDynamic;
                    //     --firstStaticAfterLastDynamic;
                    // }

                    // if (trailingStaticPoints) {
                    //     // separates trailing static points from middle dynamic points
                    //     // in order to prevent wall from being added to a moving object
                    //     for (std::vector<AbsPoint*>::iterator it = firstStaticAfterLastDynamic; it != currentGroup->points.end(); ) {
                    //         after.points.push_back(*it);
                    //         it = currentGroup->points.erase(it);
                    //     }
                    // }

                    // groups.push_back(before);   // TODO order should be before, current, after (now: current, before, after)
                    // groups.push_back(after);
                }

                groups.push_back(Group());

                //ROS_INFO("new group");
                currentGroup = &groups[groups.size() - 1];
            }

            if (currentGroup) {
                currentGroup->points.push_back(&p);
            }

            p_prev = &p;
            i = bcr::incr_overflow(i, meas.points.size());

        } while (i != startIdx);

        for (std::vector<Group>::iterator it = groups.begin(); it != groups.end(); ) {
            switch(it->points.size()) {
                case 1:
                    it->points[0]->state = AbsPoint::State::UNKNOWN;
                    /* no break */
                case 0:
                    it = groups.erase(it);
                    break;
                default:
                    if (it->isMoving()) {
                        for (AbsPoint *p : it->points) {
                            p->state = AbsPoint::State::DYNAMIC;
                            it->center += p->absPos;
                        }
                        it->center /= it->points.size();
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

int xasd = 0;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    LaserMeas meas(scan);
    meas.odom = carOdom;

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

    for(int32_t i = 0; i < meas.count; i++) {
        const radian_t angle = radian_t(scan->angle_min + scan->angle_increment * i);
        const meter_t dist = meter_t(scan->ranges[i]);

        // filters out too near objects (false detection)
        if (dist > centimeter_t(10) && !std::isinf(dist.get())) {
            const Point2m absPoint = rayDistToAbsPoint(angle, dist);
            //meas.points.push_back(absPoint);
            const Point2i absMapPoint = absMap.getNearestIndexes(absPoint);
            meas.points.push_back({ i, absPoint, absMapPoint, AbsPoint::State::UNKNOWN });
            //absMap.set(absMapPoint, AbsoluteMap::CellState::OCCUPIED);
        }
    }

    static constexpr uint32_t NUM_PREV_COMPARE = 5;

    if (prevScans.size() >= NUM_PREV_COMPARE) { // checks if enough samples have been collected

        std::vector<AbsPoint*> diffPoints;

        // checks last NUM_PREV_COMPARE measurements, if point is not present in any of them, marks it as dynamic point

        for (uint32_t i = 0; i < meas.points.size(); ++i) {

            for (uint32_t s = 1; s <= NUM_PREV_COMPARE; ++s) {
                const uint32_t id = bcr::sub_underflow(prevScans.size(), s, prevScans.size());
                const LaserMeas& prev = prevScans[id];

                // checks if correspondent point can be found in the previous measurement
                // if yes, it means point is a static point
                // if no, then dynamic

                AbsPoint& p = meas.points[i];
                //const meter_t eps = bcr::clamp(p.length() * scan->angle_increment, meter_t(0.02f), meter_t(0.2f));
                const meter_t eps = p.absPos.length() * scan->angle_increment / 4;
                bool found = false;
                for (const AbsPoint& p_prev : prev.points) {
                    if (p.absPos.distance(p_prev.absPos) < eps) {
                        found = true;
                        break;
                    }
                }

                // if point is not found in one of the previous measurements, it may be a dynamic point
                // but first checks if current distance is smaller than the previous, otherwise it is not a moving object that we have detected,
                // but the wall behind an object that is moving away
                if (!found) {

                    const meter_t cur_dist = meter_t(scan->ranges[p.idx]);
                    const meter_t prev_dist = getDistInDirection(prev, p.absPos);

                    if (cur_dist < prev_dist + centimeter_t(50)) {
                        //ROS_INFO("possible diff point: %d: (%d, %d)", p.idx, p.gridPos.X, p.gridPos.Y);
                        //diffGrid.data[p.gridPos.Y * diffGrid.info.width + p.gridPos.X] = 100;
                        p.state = AbsPoint::State::DYNAMIC;
                        diffPoints.push_back(&p);
                        break;
                    }
                }
            }
        }

        // removes noise - only keeps dynamic points that have left and right dynamic neighbours in the scan's ranges

        std::vector<AbsPoint*> filteredDiffPoints;
        filteredDiffPoints.reserve(diffPoints.size());

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
                //ROS_INFO("not diff point: %d: (%d, %d)", p.idx, p.gridPos.X, p.gridPos.Y);
                diffGrid.data[p->gridPos.Y * diffGrid.info.width + p->gridPos.X] = 0;
                p->state = AbsPoint::State::UNKNOWN;
            } else {
                //ROS_INFO("diff point: %d: (%d, %d)", p.idx, p.gridPos.X, p.gridPos.Y);
                filteredDiffPoints.push_back(p);                
            }
        }

        const std::vector<Group> groups = makeGroups(filteredDiffPoints, meas, radian_t(scan->angle_increment));

        for (const Group& g : groups) {
            if (g.isMoving()) {
                ROS_INFO("Group center: %f, %f", g.center.X.get(), g.center.Y.get());
                for (const AbsPoint *p : g.points) {
                    //ROS_INFO("    group point: %d: (%d, %d)", p.idx, p.gridPos.X, p.gridPos.Y);
                    diffGrid.data[p->gridPos.Y * diffGrid.info.width + p->gridPos.X] = 100;
                }
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
    //ROS_INFO("Car pos: [%f, %f] m, orientation: %f deg", static_cast<meter_t>(carOdom.pose.X).get(), static_cast<meter_t>(carOdom.pose.Y).get(), static_cast<radian_t>(carOdom.pose.angle).get());
}

int main(int argc, char **argv)
{
    static const std::string NODE_NAME = "environment_builder__decomposer";
    ros::init(argc, argv, NODE_NAME);
    node.reset(new DecomposerNode(NODE_NAME));
    ros::Subscriber scanSub = node->subscribe<sensor_msgs::LaserScan>(SCAN_TOPIC, 1, scanCallback);
    ros::Subscriber odomSub = node->subscribe<nav_msgs::Odometry>("odom", 1, odomCallback);

    ros::Publisher diffGridPublisher = node->advertise<nav_msgs::OccupancyGrid>("diff_grid", 10);
    diffGridPub = &diffGridPublisher;

    absMap.initialize(MAP_SIZE, MAP_RES);

    staticScan.ranges.reserve(MAX_SCAN_SAMPLES);

    dynamicObjects.reserve(50);

    ros::spin();

    return 0;
}