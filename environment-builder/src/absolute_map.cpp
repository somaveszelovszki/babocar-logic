#include <environment-builder/absolute_map.hpp>
#include <babocar-core/linalg.hpp>

#include <array>
#include <algorithm>
#include <ros/ros.h>

using namespace bcr;

void AbsoluteMap::initialize(meter_t _size, meter_t _resolution) {
    this->size_ = _size;
    this->resolution_ = _resolution;
    this->row_size_ = _size / _resolution;

    this->cells.resize(this->row_size_ * this->row_size_, CellState::UNKNOWN);
    this->tempCells.resize(this->row_size_ * this->row_size_, CellState::UNKNOWN);
    this->grid.data.resize(this->row_size_ * this->row_size_, 50);
}

void AbsoluteMap::updateCenter(const Point2m& center) {

    const int32_t cur_x = bcr::round(center.X / this->resolution_);
    const int32_t cur_y = bcr::round(center.Y / this->resolution_);

    if (cur_x != this->prev_x || cur_y != this->prev_y) {
        this->center = { cur_x * this->resolution_, cur_y * this->resolution_ };
        const int32_t dx = cur_x - this->prev_x;
        const int32_t dy = cur_y - this->prev_y;

        this->prev_x = cur_x;
        this->prev_y = cur_y;

        this->grid.info.origin.position.x = this->center.X.get();
        this->grid.info.origin.position.y = this->center.Y.get();
        this->grid.info.origin.position.z = 0;

        this->tempCells = this->cells;

        std::fill(this->cells.begin(), this->cells.end(), CellState::UNKNOWN);
        std::fill(this->grid.data.begin(), this->grid.data.end(), 50);

        for (uint32_t x = 0; x < this->row_size_; ++x) {
            for (uint32_t y = 0; y < this->row_size_; ++y) {

                const CellState state =
                    bcr::isBtw(static_cast<int32_t>(x) + dx, 0, this->row_size_ - 1) &&
                    bcr::isBtw(static_cast<int32_t>(y) + dy, 0, this->row_size_ - 1)
                    ? this->tempCells[(y + dy) * this->row_size_ + (x + dx)] : CellState::UNKNOWN;

                this->set({ x, y }, state);
            }   
        }
    }
}

AbsoluteMap::CellState AbsoluteMap::get(const Point2<uint32_t>& pos) const {
    return this->cells.at(pos.Y * this->row_size_ + pos.X);
}

void AbsoluteMap::set(const Point2<uint32_t>& pos, CellState state) {
    const uint32_t idx = pos.Y * this->row_size_ + pos.X;
    this->cells.at(idx) = state;
    this->grid.data.at(idx) = state == CellState::OCCUPIED ? 100 : state == CellState::FREE ? 0 : 50;
}

AbsoluteMap::CellState AbsoluteMap::getNearest(const Point2m& point) const {
    return this->get(this->getNearestIndexes(point));
}

void AbsoluteMap::setNearest(const Point2m& point, AbsoluteMap::CellState state) {
    this->set(this->getNearestIndexes(point), state);
}

void AbsoluteMap::setRay(const Point2m& center, const Point2m& rayEnd) {

    const Point2<uint32_t> centerPos = this->getNearestIndexes(center);
    const Point2<uint32_t> rayEndPos = this->getNearestIndexes(rayEnd);
    const radian_t angle = center.getAngle(rayEnd);
    const meter_t stepLen = this->resolution_;

    const meter_t dx = bcr::cos(angle) * stepLen, dy = bcr::sin(angle) * stepLen;
    uint32_t numSteps = (rayEnd - center).length() / stepLen;

    this->set(rayEndPos, CellState::OCCUPIED);
    Point2<uint32_t> prevPos = centerPos;
    Point2m current = center;

    uint32_t i = 0;
    do {
        const Point2<uint32_t> mapPos = this->getNearestIndexes(current);
        this->set(mapPos, CellState::FREE);

        current.X += dx;
        current.Y += dy;

        prevPos = mapPos;
    } while (prevPos != rayEndPos && ++i < numSteps);
}

void AbsoluteMap::setRay(const Point2m& center, radian_t angle) {

    const Point2<uint32_t> centerPos = this->getNearestIndexes(center);
    const meter_t stepLen = this->resolution_;

    const meter_t dx = bcr::cos(angle) * stepLen, dy = bcr::sin(angle) * stepLen;

    Point2<uint32_t> prevPos = centerPos;
    Point2m current = center;

    uint32_t i = 0;
    do {
        if (!this->isInside(current)) {
            break;
        }

        const Point2<uint32_t> mapPos = this->getNearestIndexes(current);
        this->set(mapPos, CellState::FREE);

        current.X += dx;
        current.Y += dy;

        prevPos = mapPos;
    } while (true);
}

Point2<uint32_t> AbsoluteMap::getNearestIndexes(const Point2m& point) const {

    const Point2m relPoint = point - this->center;
    const float32_t rateX = bcr::abs(relPoint.X / (this->size_ / 2));
    const float32_t rateY = bcr::abs(relPoint.Y / (this->size_ / 2));

    const Point2m insidePoint = relPoint / bcr::max(1.0f, bcr::max(rateX, rateY));

    return { 
        bcr::clamp(this->row_size_ / 2 + bcr::round(insidePoint.X / this->resolution_), 0u, this->row_size_ - 1u),
        bcr::clamp(this->row_size_ / 2 + bcr::round(insidePoint.Y / this->resolution_), 0u, this->row_size_ - 1u)
    };
}

bool AbsoluteMap::isInside(const Point2m& point) const {
    return bcr::isBtw(point.X, this->center.X - this->size_ / 2, this->center.X + this->size_ / 2) &&
        bcr::isBtw(point.Y, this->center.Y - this->size_ / 2, this->center.Y + this->size_ / 2);
}

uint32_t AbsoluteMap::numObstacleNeighbours(const Point2<uint32_t>& point, uint32_t delta) const {
    const uint32_t minX = std::max(point.X - delta, 0u);
    const uint32_t minY = std::max(point.Y - delta, 0u);
    const uint32_t maxX = std::max(static_cast<uint32_t>(point.X) + delta, this->row_size_ - 1u);
    const uint32_t maxY = std::max(static_cast<uint32_t>(point.Y) + delta, this->row_size_ - 1u);

    uint32_t count = 0;

    for (uint32_t x = minX; x < maxX; ++x) {
        for (uint32_t y = minY; y < maxY; ++y) {
            if (this->get({ x, y }) == CellState::OCCUPIED) {
                ++count;
            }
        }
    }

    return count;
}

void AbsoluteMap::filter(void) {
    static constexpr uint32_t FILTER_RADIUS = 1;

    for (uint32_t x = FILTER_RADIUS; x < this->row_size_ - FILTER_RADIUS; ++x) {
        for (uint32_t y = FILTER_RADIUS; y < this->row_size_ - FILTER_RADIUS; ++y) {
            if (this->get({ x, y }) == CellState::OCCUPIED) {
                bool found = false;
                for (uint32_t xx = x - FILTER_RADIUS; xx < x + FILTER_RADIUS; ++xx) {
                    for (uint32_t yy = y - FILTER_RADIUS; yy < y + FILTER_RADIUS; ++yy) {
                        if ((x != xx || y != yy) && this->get({ xx, yy }) == CellState::OCCUPIED) {
                            found = true;
                            break;
                        }
                    }
                }
                if (!found) {
                    this->set({ x, y }, CellState::UNKNOWN);
                }
            }
        }
    }
}