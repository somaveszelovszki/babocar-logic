#include <environment-builder/absolute_map.hpp>
#include <babocar-core/linalg.hpp>

#include <array>
#include <algorithm>

using namespace bcr;

void AbsoluteMap::initialize(meter_t _size, meter_t _resolution) {
    this->size_ = _size;
    this->resolution_ = _resolution;
    this->row_size_ = _size / _resolution;

    this->center = { this->row_size_ / 2, this->row_size_ / 2 };
    this->cells.resize(this->row_size_ * this->row_size_, CellState::UNKNOWN);
}

AbsoluteMap::CellState AbsoluteMap::get(const Point2i& pos) const {
    return this->cells.at(pos.Y * this->row_size_ + pos.X);
}

AbsoluteMap::CellState AbsoluteMap::get(int32_t x, int32_t y) const {
    return this->cells.at(y * this->row_size_ + x);
}

void AbsoluteMap::set(const Point2i& pos, CellState state) {
    this->cells.at(pos.Y * this->row_size_ + pos.X) = state;
}

AbsoluteMap::CellState AbsoluteMap::getNearest(const Point2m& point) const {
    return this->get(this->getNearestIndexes(point));
}

void AbsoluteMap::setNearest(const Point2m& point, AbsoluteMap::CellState state) {
    this->set(this->getNearestIndexes(point), state);
}

void AbsoluteMap::setRay(const Point2m& center, const Point2m& rayEnd) {
    const Point2i centerPos = this->getNearestIndexes(center);
    const Point2f rayEndPos = static_cast<Point2f>(this->getNearestIndexes(rayEnd));
    const float32_t rayLength = centerPos.distance(rayEndPos);
    this->set(rayEndPos, CellState::OCCUPIED);

    Point2i pos = centerPos;
    Point2i prevPos = pos;

    // finds closest cells to the line connecting the car and the measured point marks them as FREE
    const Line2f line(centerPos, rayEndPos);

    while(pos != rayEndPos) {
        this->set(pos, CellState::FREE);

        // finds neighbour closest to the line, that will be the next cell
        const std::array<Point2i, 4> neighbours = {
            Point2i(pos.X - 1, pos.Y),
            Point2i(pos.X + 1, pos.Y),
            Point2i(pos.X, pos.Y - 1),
            Point2i(pos.X, pos.Y + 1)
        };

        float32_t min_dist = std::numeric_limits<float32_t>::max();
        const Point2i *nextPos = nullptr;

        std::for_each(neighbours.begin(), neighbours.end(), [&](const Point2i& p) {
            if (p != prevPos) {
                const Point2f pf = static_cast<Point2f>(p);
                const float32_t dist = bcr::distance(line, pf);
                if (dist < min_dist && (pos != centerPos || pf.distance(rayEndPos) < rayLength)) {
                    min_dist = dist;
                    nextPos = &p;
                }
            }
        });

        prevPos = pos;
        pos = *nextPos;
    }
}

Point2i AbsoluteMap::getNearestIndexes(const Point2m& point) const {
    return { 
        bcr::clamp(this->center.X + bcr::round(point.X / this->resolution_), 0, this->row_size_ - 1),
        bcr::clamp(this->center.Y + bcr::round(point.Y / this->resolution_), 0, this->row_size_ - 1)
    };
}

bool AbsoluteMap::isInside(const Point2m& point) const {
    return bcr::isBtw(point.X, -this->size_ / 2, this->size_ / 2) &&
        bcr::isBtw(point.Y, -this->size_ / 2, this->size_ / 2);
}

uint32_t AbsoluteMap::numObstacleNeighbours(const Point2i& point, uint32_t delta) const {
    const int32_t minX = std::max(point.X - static_cast<int32_t>(delta), 0);
    const int32_t minY = std::max(point.Y - static_cast<int32_t>(delta), 0);
    const int32_t maxX = std::max(point.X + static_cast<int32_t>(delta), this->row_size_ - 1);
    const int32_t maxY = std::max(point.Y + static_cast<int32_t>(delta), this->row_size_ - 1);

    uint32_t count = 0;

    for (int32_t x = minX; x < maxX; ++x) {
        for (int32_t y = minY; y < maxY; ++y) {
            if (this->get(x, y) == CellState::OCCUPIED) {
                ++count;
            }
        }
    }

    return count;
}
