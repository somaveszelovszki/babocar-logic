#pragma once

#include <babocar-core/units.hpp>

namespace bcr {

struct VelocityObstacle {
    m_per_sec_t speed;
    radian_t wheelAngle;
    bool available;
};

} // namespace bcr