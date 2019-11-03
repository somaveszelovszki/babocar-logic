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

    float32_t getFactor(float32_t w_safety, float32_t w_direction, float32_t w_speed) const {
        return (this->safetyFactor * w_safety + this->directionFactor * w_direction + this->speedFactor * w_speed) / (w_safety + w_direction + w_speed);
    }
};

} // namespace bcr