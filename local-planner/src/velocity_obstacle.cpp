#include <local-planner/velocity_obstacle.hpp>

#include <babocar-core/linalg.hpp>

namespace bcr {

meter_t getDistanceToCollision(const DynamicObject& obj1, const DynamicObject& obj2) {

    const Point2m relativePosDiff = obj2.odom.pose.pos - obj1.odom.pose.pos;
    const Point2<m_per_sec_t> relativeSpeed = obj1.odom.twist.speed - obj2.odom.twist.speed;
    const Line2d relativeMovementLine(relativeSpeed.Y / relativeSpeed.X, -1.0, 0.0); // y = (vy / vx) * x

    const std::pair<Point2d, Point2d> intersections = bcr::lineCircle_intersection(
        relativeMovementLine,
        Point2d(relativePosDiff.X.get(), relativePosDiff.Y.get()),
        static_cast<float64_t>(static_cast<meter_t>(obj1.radius + obj2.radius).get())
    );

    meter_t dist = meter_t(std::numeric_limits<float64_t>::infinity());

    if (!std::isnan(intersections.first.X)) {
        dist = meter_t(bcr::pythag(intersections.first.X, intersections.first.Y));
    }

    if (!std::isnan(intersections.second.X)) {
        dist = bcr::min(dist, meter_t(bcr::pythag(intersections.second.X, intersections.second.Y)));
    }

    const m_per_sec_t v1 = obj1.odom.twist.speed.length();
    const m_per_sec_t v_rel = bcr::pythag(relativeSpeed.X, relativeSpeed.Y);

    return dist * (v1 / v_rel);
}

meter_t getDistanceToCollision_iterative(DynamicObject obj1, DynamicObject obj2, const meter_t stopDistance, const millisecond_t step) {

    const Point2m startPos = obj1.odom.pose.pos;

    const meter_t d_dist1 = obj1.odom.twist.speed.length() * step;
    const radian_t d_angle1 = obj1.odom.twist.ang_vel * step / 2;
    const float64_t sin_d_angle1 = bcr::sin(d_angle1);
    const float64_t cos_d_angle1 = bcr::cos(d_angle1);

    const radian_t d_angle2 = obj2.odom.twist.ang_vel * step / 2;
    const float64_t sin_d_angle2 = bcr::sin(d_angle2);
    const float64_t cos_d_angle2 = bcr::cos(d_angle2);

    meter_t dist1 = meter_t(0);
    bool collision = false;

    while (!collision && dist1 < stopDistance) {
        obj1.odom.twist.speed.rotate(sin_d_angle1, cos_d_angle1);
        obj1.odom.pose.pos += obj1.odom.twist.speed * step;
        obj1.odom.twist.speed.rotate(sin_d_angle1, cos_d_angle1);

        obj2.odom.twist.speed.rotate(sin_d_angle2, cos_d_angle2);
        obj2.odom.pose.pos += obj2.odom.twist.speed * step;
        obj2.odom.twist.speed.rotate(sin_d_angle2, cos_d_angle2);

        dist1 += d_dist1;

        if (obj1.odom.pose.pos.distance(obj2.odom.pose.pos) < obj1.radius + obj2.radius) {
            collision = true;
        }
    }

    if (!collision) {
        dist1 = meter_t(std::numeric_limits<float64_t>::infinity());
    }

    return dist1;
}

} // namespace bcr