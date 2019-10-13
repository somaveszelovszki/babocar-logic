#include <environment-builder/ros_convert.hpp>
#include <babocar-core/ros/ros_convert.hpp>

namespace bcr {

environment_builder::DynamicObject ros_convert(const DynamicObject& dynObj) {
    environment_builder::DynamicObject result;
    result.radius = dynObj.radius.get();
    result.pose = bcr::ros_convert(dynObj.odom.pose);
    result.twist = bcr::ros_convert(dynObj.odom.twist);
    return result;
}

DynamicObject ros_convert(const environment_builder::DynamicObject& dynObj) {
    DynamicObject result;
    result.radius = meter_t(dynObj.radius);
    result.odom.pose = bcr::ros_convert(dynObj.pose);
    result.odom.twist = bcr::ros_convert(dynObj.twist);
    return result;
}

} // namespace bcr

