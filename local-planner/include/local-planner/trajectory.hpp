#pragma once

#include <babocar-core/point2.hpp>

#include <vector>

namespace bcr {

rad_per_sec_t getAngularVelocity(meter_t carFrontRearWheelAxisDist, m_per_sec_t speed, radian_t wheelAngle);

radian_t getWheelAngle(meter_t carFrontRearWheelAxisDist, m_per_sec_t speed, rad_per_sec_t angVel);

radian_t getTrajectoryWheelAngle(meter_t carFrontRearWheelAxisDist, const Point2m currentPos, radian_t orientation, const Point2m destination);

} // namespace bcr