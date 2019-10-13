#pragma once

#include <babocar-core/odometry.hpp>

namespace bcr {

struct DynamicObject
{
    meter_t radius;  // Algorithms simplify dynamic objects to circles.
    Odometry odom;   // Moving object's odometry.
};

} // namespace bcr