#include <environment-builder/absolute_map.hpp>
#include <babocar-core/linalg.hpp>

#include <array>
#include <algorithm>

using namespace bcr;

void AbsoluteMap::initialize(distance_t _size, distance_t _resolution) {
    this->size_ = _size;
    this->resolution_ = _resolution;
    this->row_size_ = _size / _resolution;

    this->center = { this->row_size_ / 2, this->row_size_ / 2 };
    this->cells.resize(this->row_size_ * this->row_size_, CellState::UNKNOWN);
}

AbsoluteMap::CellState AbsoluteMap::get(const Point2i& pos) const {
    return this->cells.at(pos.Y * this->row_size_ + pos.X);
}

void AbsoluteMap::set(const Point2i& pos, CellState state) {
    this->cells.at(pos.Y * this->row_size_ + pos.X) = state;
}

AbsoluteMap::CellState AbsoluteMap::getNearest(const Point2<distance_t>& point) const {
    return this->get(this->getNearestIndexes(point));
}

void AbsoluteMap::setNearest(const Point2<distance_t>& point, AbsoluteMap::CellState state) {
    this->set(this->getNearestIndexes(point), state);
}

void AbsoluteMap::setRay(const Point2<distance_t>& center, const Point2<distance_t>& rayEnd) {
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

Point2i AbsoluteMap::getNearestIndexes(const Point2<distance_t>& point) const {
    return { 
        this->center.X + bcr::round(point.X / this->resolution_), 
        this->center.Y + bcr::round(point.Y / this->resolution_) 
    };
}
