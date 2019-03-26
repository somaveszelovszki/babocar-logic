#pragma once

#include <babocar-core/units.hpp>
#include <babocar-core/point2.hpp>
#include <babocar-core/pose.hpp>

#include <vector>

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

    CellState get(const Point2i& pos) const;

    CellState get(int32_t x, int32_t y) const;

    void set(const Point2i& pos, CellState state);

    CellState getNearest(const Point2m& point) const;

    void setNearest(const Point2m& point, CellState state);

    void setRay(const Point2m& center, const Point2m& rayEnd);

    Point2i getNearestIndexes(const Point2m& point) const;

    bool isInside(const Point2m& point) const;

    uint32_t numObstacleNeighbours(const Point2i& point, uint32_t delta) const;

private:
    Point2i center;
    meter_t size_;
    meter_t resolution_;
    int32_t row_size_;
    std::vector<CellState> cells;
};

} // namespace bcr