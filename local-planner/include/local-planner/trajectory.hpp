#pragma once

#include <environment-builder/dynamic_object.hpp>

#include <vector>

namespace bcr {

struct ObjectTrajectory {
    meter_t radius;
    millisecond_t step;
    std::vector<Point2m> points;
};

meter_t getRadius(meter_t carFrontRearWheelAxisDist, radian_t wheelAngle);

meter_t getRadius(m_per_sec_t speed, rad_per_sec_t angVel);

Sign getSpeedSign(const DynamicObject& obj);

rad_per_sec_t getAngularVelocity(meter_t carFrontRearWheelAxisDist, m_per_sec_t speed, radian_t wheelAngle);

radian_t getWheelAngle(meter_t carFrontRearWheelAxisDist, meter_t radius);

radian_t getWheelAngle(meter_t carFrontRearWheelAxisDist, m_per_sec_t speed, rad_per_sec_t angVel);

radian_t getTrajectoryWheelAngle(meter_t carFrontRearWheelAxisDist, const Point2m currentPos, radian_t orientation, const Point2m destination);

ObjectTrajectory getTrajectory(const DynamicObject& obj, const millisecond_t timeInterval, const millisecond_t step);

} // namespace bcr