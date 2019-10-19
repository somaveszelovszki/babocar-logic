#pragma once

#include <environment-builder/dynamic_object.hpp>

namespace bcr {

meter_t getDistanceToCollision(const DynamicObject& obj1, const DynamicObject& obj2);

meter_t getDistanceToCollision_iterative(DynamicObject obj1, DynamicObject obj2, const meter_t stopDistance, const millisecond_t step);

} // namespace bcr