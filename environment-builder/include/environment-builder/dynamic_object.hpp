#pragma once

#include <babocar-core/types.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

namespace bcr {

struct DynamicObject
{
    uint32_t id;
    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;
};

} // namespace bcr