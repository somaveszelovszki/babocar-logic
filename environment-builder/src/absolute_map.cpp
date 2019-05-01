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
    this->grid.data.resize(this->row_size_ * this->row_size_, 50);
}

AbsoluteMap::CellState AbsoluteMap::get(const Point2i& pos) const {
    return this->cells.at(pos.Y * this->row_size_ + pos.X);
}

AbsoluteMap::CellState AbsoluteMap::get(int32_t x, int32_t y) const {
    return this->cells.at(y * this->row_size_ + x);
}

void AbsoluteMap::set(const Point2i& pos, CellState state) {
    this->cells.at(pos.Y * this->row_size_ + pos.X) = state;
    this->grid.data.at(pos.Y * this->row_size_ + pos.X) = state == CellState::OCCUPIED ? 100 : state == CellState::FREE ? 0 : 50;
}

AbsoluteMap::CellState AbsoluteMap::getNearest(const Point2m& point) const {
    return this->get(this->getNearestIndexes(point));
}

void AbsoluteMap::setNearest(const Point2m& point, AbsoluteMap::CellState state) {
    this->set(this->getNearestIndexes(point), state);
}

void AbsoluteMap::setRay(const Point2m& center, const Point2m& rayEnd) {

    const Point2i centerPos = this->getNearestIndexes(center);
    const Point2i rayEndPos = this->getNearestIndexes(rayEnd);
    const radian_t angle = center.getAngle(rayEnd);
    const meter_t stepLen = this->resolution_;

    const meter_t dx = bcr::cos(angle) * stepLen, dy = bcr::sin(angle) * stepLen;
    uint32_t numSteps = (rayEnd - center).length() / stepLen;

    this->set(rayEndPos, CellState::OCCUPIED);
    Point2i prevPos = centerPos;
    Point2m current = center;

    uint32_t i = 0;
    do {
        const Point2i mapPos = this->getNearestIndexes(current);
        this->set(mapPos, CellState::FREE);

        current.X += dx;
        current.Y += dy;

        prevPos = mapPos;
    } while (prevPos != rayEndPos && ++i < numSteps);
}

void AbsoluteMap::setRay(const Point2m& center, radian_t angle) {

    const Point2i centerPos = this->getNearestIndexes(center);
    const meter_t stepLen = this->resolution_;

    const meter_t dx = bcr::cos(angle) * stepLen, dy = bcr::sin(angle) * stepLen;

    Point2i prevPos = centerPos;
    Point2m current = center;

    uint32_t i = 0;
    do {
        if (!this->isInside(current)) {
            break;
        }

        const Point2i mapPos = this->getNearestIndexes(current);
        this->set(mapPos, CellState::FREE);

        current.X += dx;
        current.Y += dy;

        prevPos = mapPos;
    } while (true);
}

Point2i AbsoluteMap::getNearestIndexes(const Point2m& point) const {

    const float32_t rateX = bcr::abs(point.X / (this->size_ / 2));
    const float32_t rateY = bcr::abs(point.Y / (this->size_ / 2));

    const Point2m insidePoint = point / bcr::max(1.0f, bcr::max(rateX, rateY));

    return { 
        bcr::clamp(this->center.X + bcr::round(insidePoint.X / this->resolution_), 0, this->row_size_ - 1),
        bcr::clamp(this->center.Y + bcr::round(insidePoint.Y / this->resolution_), 0, this->row_size_ - 1)
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
