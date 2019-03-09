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

    void initialize(distance_t _size, distance_t _resolution);

    CellState get(const Point2<int32_t>& pos) const;

    void set(const Point2<int32_t>& pos, CellState state);

    CellState getNearest(const Point2<distance_t>& point) const;

    void setNearest(const Point2<distance_t>& point, CellState state);

    void setRay(const Point2<distance_t>& center, const Point2<distance_t>& rayEnd);

private:

    Point2<int32_t> getNearestIndexes(const Point2<distance_t>& point) const;

    Point2<int32_t> center;
    distance_t size_;
    distance_t resolution_;
    int32_t row_size_;
    std::vector<CellState> cells;
};

} // namespace bcr