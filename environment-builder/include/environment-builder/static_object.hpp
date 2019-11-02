#pragma once

#include <babocar-core/point2.hpp>

namespace bcr {

struct StaticObject
{
    meter_t radius;  // Algorithms simplify dynamic objects to circles.
    Point2m pos;     // Object's position.
};

} // namespace bcr