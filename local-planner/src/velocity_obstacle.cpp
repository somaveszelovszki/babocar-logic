#include <local-planner/velocity_obstacle.hpp>

#include <babocar-core/linalg.hpp>

namespace bcr {

meter_t getDistanceToCollision(const DynamicObject& obj1, const DynamicObject& obj2) {

    const Point2m relativePosDiff = obj2.odom.pose.pos - obj1.odom.pose.pos;
    const Point2<m_per_sec_t> relativeSpeed(obj1.odom.twist.X - obj2.odom.twist.X, obj1.odom.twist.Y - obj2.odom.twist.Y);
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

    const m_per_sec_t v1 = bcr::pythag(obj1.odom.twist.X, obj1.odom.twist.Y);
    const m_per_sec_t v_rel = bcr::pythag(relativeSpeed.X, relativeSpeed.Y);

    return dist * (v1 / v_rel);
}

} // namespace bcr