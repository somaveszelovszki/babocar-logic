#include <local-planner/trajectory.hpp>

#include <babocar-core/unit_utils.hpp>
#include <babocar-core/linalg.hpp>

namespace bcr {

meter_t getRadius(meter_t carFrontRearWheelAxisDist, radian_t wheelAngle) {
    return isZero(wheelAngle) ? meter_t(std::numeric_limits<unit_storage_type>::infinity()) : carFrontRearWheelAxisDist / -bcr::tan(wheelAngle);
}

meter_t getRadius(m_per_sec_t speed, rad_per_sec_t angVel) {
    return speed / -angVel;
}

Sign getSpeedSign(const DynamicObject& obj) {
    return eq(obj.odom.twist.speed.getAngle(), obj.odom.pose.angle, PI_2) ? Sign::POSITIVE : Sign::NEGATIVE;
}

rad_per_sec_t getAngularVelocity(meter_t carFrontRearWheelAxisDist, m_per_sec_t speed, radian_t wheelAngle) {
    rad_per_sec_t angVel(0);
    if (!isZero(wheelAngle)) {
        const meter_t R = getRadius(carFrontRearWheelAxisDist, wheelAngle);
        angVel = speed / -R;
    }
    return angVel;
}

radian_t getWheelAngle(meter_t carFrontRearWheelAxisDist, m_per_sec_t speed, rad_per_sec_t angVel) {
    radian_t wheelAngle(0);
    if (!isZero(angVel)) {
        const meter_t R = speed / -angVel;
        wheelAngle = atan(carFrontRearWheelAxisDist / -R);
    }
    return wheelAngle;
}

radian_t getTrajectoryWheelAngle(meter_t carFrontRearWheelAxisDist, const Point2m currentPos, radian_t orientation, const Point2m destination) {

    const float64_t x1 = currentPos.X.get();
    const float64_t y1 = currentPos.Y.get();
    const float64_t x2 = destination.X.get();
    const float64_t y2 = destination.Y.get();

    const float64_t x12 = avg(x1, x2);
    const float64_t y12 = avg(y1, y2);

    const float64_t tgAlpha = tan(orientation - PI_2);
    const float64_t tgBeta = (x1 - x2) / (y2 - y1);

    const Line2d e = { tgAlpha, -1.0, y1 - x1 * tgAlpha };
    const Line2d f = { tgBeta, -1.0, y12 - x12 * tgBeta };

    const Point2d intersection = lineLine_intersection(e, f);

    radian_t wheelAngle = radian_t::ZERO();

    if (!std::isinf(intersection.X) && !std::isinf(intersection.Y)) {
        const meter_t R = currentPos.distance(static_cast<Point2m>(intersection));
        wheelAngle = atan(carFrontRearWheelAxisDist / R);
    }

    return wheelAngle;
}

ObjectTrajectory getTrajectory(const DynamicObject& obj, const millisecond_t timeInterval, const millisecond_t step) {
    ObjectTrajectory traj;
    traj.radius = obj.radius;
    traj.step = step;

    traj.points.reserve(static_cast<size_t>(timeInterval / step) + 1);

    Odometry odom = obj.odom;
    millisecond_t time = millisecond_t(0);

    while (time < timeInterval) {
        traj.points.push_back(odom.pose.pos);
        odom.update(step);
        time += step;
    }

    return traj;
}

} // namespace bcr