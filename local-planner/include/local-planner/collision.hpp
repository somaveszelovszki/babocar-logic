#pragma once

#include <environment-builder/static_object.hpp>
#include "trajectory.hpp"

#include <vector>

namespace bcr {

millisecond_t getTimeToFirstCollision_iterative(
    DynamicObject obj, const std::vector<StaticObject>& staticObjects, const std::vector<ObjectTrajectory>& trajectories, const millisecond_t timeInterval, const millisecond_t step);

} // namespace bcr