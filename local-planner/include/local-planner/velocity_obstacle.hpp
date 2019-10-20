#pragma once

#include <environment-builder/dynamic_object.hpp>

#include <vector>

namespace bcr {

meter_t getDistanceToCollision(const DynamicObject& obj1, const DynamicObject& obj2);

meter_t getDistanceToCollision_iterative(DynamicObject obj1, DynamicObject obj2, const meter_t stopDistance, const millisecond_t step);

meter_t getDistanceToFirstCollision_iterative(DynamicObject obj1, std::vector<DynamicObject> objects, const meter_t stopDistance, const millisecond_t step);

} // namespace bcr