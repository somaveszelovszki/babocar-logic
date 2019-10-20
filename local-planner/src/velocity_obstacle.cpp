#include <local-planner/velocity_obstacle.hpp>

#include <babocar-core/linalg.hpp>

namespace bcr {

struct diff {
    millisecond_t step;
    meter_t dist;
    radian_t angle_per_2;
    float64_t angle_per_2_sine;
    float64_t angle_per_2_cosine;

    diff(const Point2mps& speed, rad_per_sec_t ang_vel, millisecond_t step)
        : step(step)
        , dist(speed.length() * step)
        , angle_per_2(ang_vel * step / 2)
        , angle_per_2_sine(bcr::sin(angle_per_2))
        , angle_per_2_cosine(bcr::cos(angle_per_2)) {}

    void apply(Odometry& odom, meter_t *dist) {
        odom.twist.speed.rotate(this->angle_per_2_sine, this->angle_per_2_cosine);
        odom.pose.pos += odom.twist.speed * this->step;
        odom.twist.speed.rotate(this->angle_per_2_sine, this->angle_per_2_cosine);

        if (dist) {
            *dist += this->dist;
        }
    }
};

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
    return getDistanceToFirstCollision_iterative(obj1, { obj2 }, stopDistance, step);
}

meter_t getDistanceToFirstCollision_iterative(DynamicObject obj1, std::vector<DynamicObject> objects, const meter_t stopDistance, const millisecond_t step) {
    
    const Point2m startPos = obj1.odom.pose.pos;

    diff diff1(obj1.odom.twist.speed, obj1.odom.twist.ang_vel, step);

    std::vector<diff> diffs;
    diffs.reserve(objects.size());
    for (const DynamicObject& obj : objects) {
        diffs.emplace_back(obj.odom.twist.speed, obj.odom.twist.ang_vel, step);
    }

    meter_t dist1 = meter_t(0);
    bool collision = false;

    while (!collision && dist1 < stopDistance) {

        diff1.apply(obj1.odom, &dist1);

        for (size_t i = 0; i < objects.size(); ++i) {
            diffs[i].apply(objects[i].odom, nullptr);
        }

        for(const DynamicObject& obj : objects) {
            if (obj1.odom.pose.pos.distance(obj.odom.pose.pos) < obj1.radius + obj.radius) {
                collision = true;
                break;
            }
        }
    }

    if (!collision) {
        dist1 = meter_t(std::numeric_limits<float64_t>::infinity());
    }

    return dist1;
}

} // namespace bcr