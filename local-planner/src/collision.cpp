#include <local-planner/collision.hpp>

#include <babocar-core/linalg.hpp>

#include <stdexcept>

namespace bcr {

millisecond_t getTimeToFirstCollision_iterative(
    DynamicObject obj, const std::vector<StaticObject>& staticObjects, const std::vector<ObjectTrajectory>& trajectories, const millisecond_t timeInterval, const millisecond_t step) {
    
    const m_per_sec_t speed = obj.odom.twist.speed.length();
    millisecond_t staticCollisionTime = timeInterval;

    if (staticObjects.size() > 0) {
        if (isZero(obj.odom.twist.ang_vel)) { // straight
            const Line2d line(Point2d(obj.odom.pose.pos.X.get(), obj.odom.pose.pos.Y.get()), obj.odom.pose.angle);

            for (const StaticObject& o : staticObjects) {
                const std::pair<Point2d, Point2d> intersections = lineCircle_intersection(line, { o.pos.X.get(), o.pos.Y.get() }, (obj.radius + o.radius).get());
                if (!std::isnan(intersections.first.X) && !std::isnan(intersections.first.Y) && !std::isnan(intersections.second.X) && !std::isnan(intersections.second.Y)) {
                    const Point2m p1 = static_cast<Point2m>(intersections.first);
                    const Point2m p2 = static_cast<Point2m>(intersections.second);

                    const meter_t collisionDist = min(obj.odom.pose.pos.distance(p1), obj.odom.pose.pos.distance(p2));
                    const millisecond_t collisionTime = collisionDist / speed;
                    if (collisionTime < staticCollisionTime) {
                        staticCollisionTime = collisionTime;
                    }
                }
            }

        } else { // curve
            const meter_t R = abs(speed / obj.odom.twist.ang_vel);
            const Point2m center = obj.odom.pose.pos + Vec2m(R, meter_t(0)).rotate(obj.odom.pose.angle - PI_2);
            const radian_t angle = center.getAngle(obj.odom.pose.pos);

            for (const StaticObject& o : staticObjects) {
                const meter_t dist = center.distance(o.pos);
                const meter_t r = obj.radius + o.radius;

                if (isBtw(dist, R - r, R + r)) {
                    const radian_t collisionAngle = center.getAngle(o.pos) - angle;
                    if (sgn(collisionAngle) == sgn(obj.odom.twist.ang_vel)) { // if signs do not match, then the main object is going to the wrong direction on the circle, they will not collide
                        const meter_t len = R * abs(collisionAngle);
                        const meter_t correction = r * bcr::cos(PI_2 * (abs(dist - R) / r));

                        const meter_t collisionDist = len - correction;
                        const millisecond_t collisionTime = collisionDist / speed;
                        if (collisionTime < staticCollisionTime) {
                            staticCollisionTime = collisionTime;
                        }
                    }
                }
            }
        }
    }

    millisecond_t dynamicCollisionTime(0);

    if (trajectories.size() > 0) {

        for(const ObjectTrajectory& traj : trajectories) {
            if (traj.step != step) {
                throw std::runtime_error("Trajectory step must be " + std::to_string(step.get()) + "ms (current: " + std::to_string(traj.step.get()) + "ms)");
            }
        }

        const Point2m startPos = obj.odom.pose.pos;
        bool collision = false;
        size_t i = 0;

        while (!collision && dynamicCollisionTime < timeInterval) {
    
            for(const ObjectTrajectory& traj : trajectories) {
                if (obj.odom.pose.pos.distance(traj.points[i]) < obj.radius + traj.radius) {
                    collision = true;
                    break;
                }
            }
    
            dynamicCollisionTime += step;
            obj.odom.update(step);
            ++i;
        }
    } else {
        dynamicCollisionTime = timeInterval;
    }

    return min(staticCollisionTime, dynamicCollisionTime);
}

} // namespace bcr