#pragma once

#include <babocar-core/odometry.hpp>

namespace bcr {

struct DynamicObject
{
    uint32_t id;        // Object identifier.
    distance_t radius;  // Algorithms simplify dynamic objects to circles.
    Odometry odom;      // Moving object's odometry.
};

} // namespace bcr