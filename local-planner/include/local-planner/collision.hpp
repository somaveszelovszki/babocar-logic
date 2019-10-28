#pragma once

#include <environment-builder/dynamic_object.hpp>

#include <vector>

namespace bcr {

meter_t getDistanceToCollision(const DynamicObject& obj1, const DynamicObject& obj2);

millisecond_t getTimeToFirstCollision_iterative(DynamicObject obj1, std::vector<DynamicObject> objects, const millisecond_t timeInterval, const millisecond_t step);

} // namespace bcr