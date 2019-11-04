#include <environment-builder/ros_convert.hpp>
#include <babocar-core/ros/ros_convert.hpp>

namespace bcr {

environment_builder::DynamicObject ros_convert(const DynamicObject& dynObj) {
    environment_builder::DynamicObject result;
    result.radius = dynObj.radiuses[0].second.get();
    result.pose = bcr::ros_convert(dynObj.odom.pose);
    result.twist = bcr::ros_convert(dynObj.odom.twist);
    return result;
}

DynamicObject ros_convert(const environment_builder::DynamicObject& dynObj) {
    return DynamicObject(meter_t(dynObj.radius), { bcr::ros_convert(dynObj.pose), bcr::ros_convert(dynObj.twist) });
}

} // namespace bcr

