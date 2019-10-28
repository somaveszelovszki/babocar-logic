#pragma once

#include "velocity_obstacle.hpp"

#include <vector>

namespace bcr {

class DynamicWindow {
public:
    DynamicWindow(
        m_per_sec_t maxSpeedBwd, m_per_sec_t maxSpeedFwd, radian_t maxWheelAngle, // absolute limits
        m_per_sec_t maxDeltaSpeed, radian_t maxDeltaWheelAngle, // dynamic limits
        m_per_sec_t wRes_speed, radian_t wRes_wheelAngle // window properties
    );

    void update(m_per_sec_t actualSpeed, radian_t actualWheelAngle);

    std::vector<std::vector<VelocityObstacle>>& getWindow(void) { return this->window; }

private:
    // absolute limits
    const m_per_sec_t maxSpeedBwd;
    const m_per_sec_t maxSpeedFwd;
    const radian_t maxWheelAngle;

    // dynamic limits
    const m_per_sec_t maxDeltaSpeed;
    const radian_t maxDeltaWheelAngle;

    // window properties
    std::vector<std::vector<VelocityObstacle>> window; // the window of permitted speed and angular velocity values - rows: speeds, columns: wheel angles
    const m_per_sec_t wRes_speed; 
    const radian_t wRes_wheelAngle;

    m_per_sec_t wCenter_speed;
    size_t wCenter_speedIdx;

    radian_t wCenter_wheelAngle;
    size_t wCenter_wheelAngleIdx;
};

} // namespace bcr