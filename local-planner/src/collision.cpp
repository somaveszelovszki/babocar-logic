#include <local-planner/collision.hpp>

#include <babocar-core/linalg.hpp>

#include <stdexcept>

namespace bcr {

millisecond_t getTimeToFirstCollision_iterative(
    DynamicObject obj, const std::vector<StaticObject>& staticObjects, const std::vector<ObjectTrajectory>& trajectories, const millisecond_t timeInterval, const millisecond_t step) {
    
    const Sign speedSign = getSpeedSign(obj);
    const m_per_sec_t speed = speedSign * obj.odom.twist.speed.length();
    millisecond_t staticCollisionTime = timeInterval;

    if (staticObjects.size() > 0 && !isZero(speed)) {

        for (size_t r = 0; r < obj.positions.size(); ++r) {
            //const Point2m pos = speedSign == Sign::POSITIVE ? obj.positions[obj.positions.size() - 1] : obj.positions[0];
            //const meter_t radius = speedSign == Sign::POSITIVE ? obj.radiuses[obj.radiuses.size() - 1].second : obj.radiuses[0].second;

            const Point2m pos = obj.positions[r];
            const meter_t radius = obj.radiuses[r].second;

            if (eq(obj.odom.twist.ang_vel, rad_per_sec_t(0), deg_per_sec_t(1))) { // straight
                const Line2d line(Point2d(pos.X.get(), pos.Y.get()), obj.odom.pose.angle);

                for (const StaticObject& o : staticObjects) {
                    const std::pair<Point2d, Point2d> intersections = lineCircle_intersection(line, { o.pos.X.get(), o.pos.Y.get() }, (radius + o.radius).get());
                    if (!std::isnan(intersections.first.X) && !std::isnan(intersections.first.Y) && !std::isnan(intersections.second.X) && !std::isnan(intersections.second.Y)) {
                        const Point2m p1 = static_cast<Point2m>(intersections.first);
                        const Point2m p2 = static_cast<Point2m>(intersections.second);
    
                        const meter_t dist1 = pos.distance(p1);
                        const meter_t dist2 = pos.distance(p2);
    
                        const Point2m collisionPoint = dist1 < dist2 ? p1 : p2;
    
                        // if angles do not match, then the main object is going to the wrong direction on the circle, they will not collide
                        if (eq(pos.getAngle(collisionPoint), obj.odom.twist.speed.getAngle(), PI_2)) {
                            const meter_t collisionDist = dist1 < dist2 ? dist1 : dist2;
                            const millisecond_t collisionTime = collisionDist / abs(speed);
                            if (collisionTime < staticCollisionTime) {
                                staticCollisionTime = collisionTime;
                            }
                        }
                    }
                }

            } else { // curve
                const meter_t R_signed_fromBaseLink = getRadius(speed, obj.odom.twist.ang_vel);
                const Point2m center = obj.odom.pose.pos + Vec2m(meter_t(0), -R_signed_fromBaseLink).rotate(obj.odom.pose.angle);

                const meter_t R = center.distance(pos);
                const radian_t angle = center.getAngle(pos);

                for (const StaticObject& o : staticObjects) {
                    const meter_t dist = center.distance(o.pos);
                    const meter_t radiusSum = radius + o.radius;

                    if (isBtw(dist, R - radiusSum, R + radiusSum)) {
                        const radian_t collisionAngle = normalize360(center.getAngle(o.pos) - angle + PI) - PI;
    
                        // if signs do not match, then the main object is going to the wrong direction on the circle, they will not collide
                        if (sgn(collisionAngle) == sgn(obj.odom.twist.ang_vel)) {
                            const meter_t len = abs(R * collisionAngle);
                            const meter_t correction = radiusSum * bcr::cos(PI_2 * (abs(dist - R) / radiusSum));
    
                            const meter_t collisionDist = max(len - correction, meter_t(0));
                            const millisecond_t collisionTime = collisionDist / abs(speed);
                            if (collisionTime < staticCollisionTime) {
                                staticCollisionTime = collisionTime;
                            }
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

        while (dynamicCollisionTime < timeInterval) {
    
            for(const ObjectTrajectory& traj : trajectories) {
                for (size_t r = 0; r < obj.radiuses.size(); ++r) {
                    if (obj.positions[r].distance(traj.points.at(i)) < obj.radiuses[r].second + traj.radius) {
                        collision = true;
                        break;
                    }
                }

                if (collision) {
                    break;
                }
            }

            if (collision) {
                break;
            }
    
            dynamicCollisionTime += step;
            obj.update(step);
            ++i;
        }
    } else {
        dynamicCollisionTime = timeInterval;
    }

    return min(staticCollisionTime, dynamicCollisionTime);
}

} // namespace bcr