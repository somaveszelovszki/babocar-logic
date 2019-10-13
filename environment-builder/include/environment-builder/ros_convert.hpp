#pragma once

#include <environment-builder/dynamic_object.hpp>

#include <ros/ros.h>
#include <environment_builder/DynamicObject.h>

namespace bcr {

environment_builder::DynamicObject ros_convert(const DynamicObject& dynObj);
DynamicObject ros_convert(const environment_builder::DynamicObject& dynObj);

} // namespace bcr