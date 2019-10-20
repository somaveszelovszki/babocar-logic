#include <local-planner/trajectory.hpp>

#include <babocar-core/unit_utils.hpp>

namespace bcr {

rad_per_sec_t getAngularVelocity(meter_t carFrontRearWheelAxisDist, m_per_sec_t speed, radian_t wheelAngle) {
    rad_per_sec_t angVel(0);
    if (!bcr::isZero(wheelAngle)) {
        const meter_t R = carFrontRearWheelAxisDist / bcr::tan(wheelAngle);
        angVel = speed / R;
    }
    return angVel;
}

radian_t getWheelAngle(meter_t carFrontRearWheelAxisDist, m_per_sec_t speed, rad_per_sec_t angVel) {
    radian_t wheelAngle(0);
    if (!bcr::isZero(angVel)) {
        const meter_t R = speed / angVel;
        wheelAngle = bcr::atan(carFrontRearWheelAxisDist / R);
    }
    return wheelAngle;
}

} // namespace bcr