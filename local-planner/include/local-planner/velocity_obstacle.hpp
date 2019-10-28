#pragma once

#include <babocar-core/units.hpp>

namespace bcr {

struct VelocityObstacle {
    m_per_sec_t speed;
    radian_t wheelAngle;
    bool available;
    float32_t safetyFactor;    // [0 - 1] 0: unavoidable collision, 1: safe velocity
    float32_t directionFactor; // [0 - 1] 0: furthest from desired direction, 1: exactly desired direction
    float32_t speedFactor;     // [0 - 1] 0: furthest from desired speed, 1: exactly desired speed
};

} // namespace bcr