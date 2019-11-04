#pragma once

#include <babocar-core/odometry.hpp>

#include <vector>

namespace bcr {

struct DynamicObject
{
    std::vector<std::pair<meter_t, meter_t>> radiuses;  // Algorithms simplify dynamic objects to multiple circles. Each circle has a distance from the bas_lnik, and a radius.
    std::vector<Point2m> positions;
    Odometry odom;   // Moving object's odometry.

    DynamicObject() {}

    DynamicObject(const std::vector<std::pair<meter_t, meter_t>>& radiuses, const Odometry& odom)
        : radiuses(radiuses)
        , odom(odom) {

        this->positions.resize(this->radiuses.size());
        this->updateCirclePositions();
    }

    DynamicObject(meter_t radius, const Odometry& odom) : DynamicObject({ { meter_t(0), radius } }, odom) {}

    void update(millisecond_t step) {
        this->odom.update(step);
        this->updateCirclePositions();
    }

    void updateCirclePositions() {
        const Vec2d dir = Vec2d::normVec(this->odom.pose.angle);
        for (size_t i = 0; i < this->radiuses.size(); ++i) {
            this->positions[i] = this->odom.pose.pos + this->radiuses[i].first * dir;
        }
    }
};

} // namespace bcr