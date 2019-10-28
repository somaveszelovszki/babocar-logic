#include <local-planner/dynamic_window.hpp>
#include <babocar-core/unit_utils.hpp>

#include <algorithm>
#include <limits>

namespace bcr {

DynamicWindow::DynamicWindow(
        m_per_sec_t maxSpeedBwd, m_per_sec_t maxSpeedFwd, radian_t maxWheelAngle,
        m_per_sec_t maxDeltaSpeed, radian_t maxDeltaWheelAngle,
        m_per_sec_t wRes_speed, radian_t wRes_wheelAngle)
    : maxSpeedBwd(maxSpeedBwd)
    , maxSpeedFwd(maxSpeedFwd)
    , maxWheelAngle(maxWheelAngle)
    , maxDeltaSpeed(maxDeltaSpeed)
    , maxDeltaWheelAngle(maxDeltaWheelAngle)
    , wRes_speed(wRes_speed)
    , wRes_wheelAngle(wRes_wheelAngle)
    , wCenter_speed(0)
    , wCenter_wheelAngle(0) {

    const size_t size_speed = 2 * static_cast<size_t>(maxDeltaSpeed / wRes_speed) + 1;
    const size_t size_wheelAngle = 2 * static_cast<size_t>(maxDeltaWheelAngle / wRes_wheelAngle) + 1;

    this->window.resize(size_speed);
    for (std::vector<VelocityObstacle>& col : this->window) {
        col.resize(size_wheelAngle);
    }

    this->wCenter_speedIdx = size_speed / 2;
    this->wCenter_wheelAngleIdx = size_wheelAngle / 2;

    this->update(m_per_sec_t(0), radian_t(0));
}

void DynamicWindow::update(m_per_sec_t actualSpeed, radian_t actualWheelAngle) {
    this->wCenter_speed = actualSpeed;
    this->wCenter_wheelAngle = actualWheelAngle;

    const size_t rowIdx_center = (this->window.size() + 1) / 2;
    const size_t colIdx_center = (this->window[0].size() + 1) / 2;

    for (size_t r = 0; r < this->window.size(); ++r) {
        std::vector<VelocityObstacle>& col = this->window[r];
        for (size_t c = 0; c < col.size(); ++c) {
            const m_per_sec_t speed = this->wCenter_speed + (static_cast<int32_t>(r) - static_cast<int32_t>(this->wCenter_speedIdx)) * this->wRes_speed;
            const radian_t wheelAngle = this->wCenter_wheelAngle + (static_cast<int32_t>(c) - static_cast<int32_t>(this->wCenter_wheelAngleIdx)) * this->wRes_wheelAngle;

            col[c] = { 
                speed, wheelAngle,
                bcr::isBtw(speed, this->maxSpeedBwd - m_per_sec_t(0.0001), this->maxSpeedFwd + m_per_sec_t(0.00001)) &&
                bcr::isBtw(wheelAngle, -this->maxWheelAngle - radian_t(0.00001), this->maxWheelAngle + radian_t(0.00001)),
                0.0f, 0.0f, 0.0f // factors
            };
        }
    }
}

} // namespace bcr