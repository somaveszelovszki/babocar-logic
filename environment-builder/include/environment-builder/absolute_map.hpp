#pragma once

#include <babocar-core/units.hpp>
#include <babocar-core/point2.hpp>
#include <babocar-core/pose.hpp>

#include <vector>

#include <nav_msgs/OccupancyGrid.h>

namespace bcr {

class AbsoluteMap
{
public:
    enum class CellState : int8_t {
        UNKNOWN = -1,
        FREE = 0,
        OCCUPIED = 1
    };

    void initialize(meter_t _size, meter_t _resolution);

    void updateCenter(const Point2m& center);

    CellState getNearest(const Point2m& point) const;

    void setNearest(const Point2m& point, CellState state);

    void setRay(const Point2m& center, const Point2m& rayEnd);

    void setRay(const Point2m& center, radian_t angle);

    Point2<uint32_t> getNearestIndexes(const Point2m& point) const;

    bool isInside(const Point2m& point) const;

    uint32_t numObstacleNeighbours(const Point2<uint32_t>& point, uint32_t delta) const;

    void filter(void);

    size_t getWidth(void) const { return this->row_size_; }
    size_t getHeight(void) const { return this->row_size_; }
    float32_t getResolution(void) const { return this->resolution_.get(); }

    nav_msgs::OccupancyGrid grid;   // TODO use only one implementation (this or own)

public:
    CellState get(const Point2<uint32_t>& pos) const;

    void set(const Point2<uint32_t>& pos, CellState state);

    Point2m center;
    meter_t size_;
    meter_t resolution_;
    uint32_t row_size_;
    std::vector<CellState> cells;
    std::vector<CellState> tempCells;

    int32_t prev_x;
    int32_t prev_y;
};

} // namespace bcr