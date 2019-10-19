#pragma once

#include <environment-builder/dynamic_object.hpp>

namespace bcr {

meter_t getDistanceToCollision(const DynamicObject& obj1, const DynamicObject& obj2);

} // namespace bcr