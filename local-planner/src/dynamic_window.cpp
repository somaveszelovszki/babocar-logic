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
    , wMin_speed(-maxDeltaSpeed)
    , wMax_speed(maxDeltaSpeed)
    , wMin_wheelAngle(-maxDeltaWheelAngle)
    , wMax_wheelAngle(maxDeltaWheelAngle) {

    this->window.resize(2 * maxDeltaSpeed / wRes_speed);
    for (std::vector<VelocityObstacle>& col : this->window) {
        col.resize(2 * maxDeltaWheelAngle / wRes_wheelAngle);
    }

    this->update(m_per_sec_t(0), radian_t(0));
}

void DynamicWindow::update(m_per_sec_t actualSpeed, radian_t actualWheelAngle) {
    this->wMin_speed = actualSpeed - this->maxDeltaSpeed;
    this->wMax_speed = actualSpeed + this->maxDeltaSpeed;

    this->wMin_wheelAngle = actualWheelAngle - this->maxDeltaWheelAngle;
    this->wMax_wheelAngle = actualWheelAngle + this->maxDeltaWheelAngle;

    for (size_t r = 0; r < this->window.size(); ++r) {
        std::vector<VelocityObstacle>& col = this->window[r];
        for (size_t c = 0; c < col.size(); ++c) {
            std::pair<m_per_sec_t, radian_t> dynamics = this->getDynamics(r, c);

            col[c] = { 
                dynamics.first, dynamics.second,
                bcr::isBtw(dynamics.first, this->maxSpeedBwd, this->maxSpeedFwd) &&
                bcr::isBtw(dynamics.second, -this->maxWheelAngle, this->maxWheelAngle)
            };
        }
    }
}

bool DynamicWindow::isAvailable(m_per_sec_t speed, radian_t wheelAngle) const {
    bool available = false;

    if (bcr::isBtw(speed, this->maxSpeedBwd, this->maxSpeedFwd) &&
        bcr::isBtw(wheelAngle, -this->maxWheelAngle, this->maxWheelAngle)) {

        std::pair<size_t, size_t> indexes = this->getIndexes(speed, wheelAngle);
        if (bcr::isBtw(indexes.first, 0, this->window.size() - 1)) {

            const std::vector<VelocityObstacle>& col = this->window[indexes.first];
            if (bcr::isBtw(indexes.second, 0, col.size() - 1)) {
                available = col[indexes.second].available;
            }
        }
    }
    return available;
}

std::pair<size_t, size_t> DynamicWindow::getIndexes(m_per_sec_t speed, radian_t wheelAngle) const {
    const float32_t idxSpeed = bcr::isBtw(speed, this->wMin_speed, this->wMax_speed) ? bcr::map(speed, this->wMin_speed, this->wMax_speed, 0.0f, static_cast<float32_t>(window.size() - 1)) : -1.0f;
    const float32_t idxWheelAngle = bcr::isBtw(wheelAngle, this->wMin_wheelAngle, this->wMax_wheelAngle) ? bcr::map(wheelAngle, this->wMin_wheelAngle, this->wMax_wheelAngle, 0.0f, static_cast<float32_t>(window[0].size() - 1)) : -1.0f;

    return idxWheelAngle >= 0.0f && idxSpeed >= 0.0f ?
        std::pair<size_t, size_t> { static_cast<size_t>(bcr::round(idxSpeed)), static_cast<size_t>(bcr::round(idxWheelAngle)) } :
        std::pair<size_t, size_t> { size_t(-1), size_t(-1) };
}

std::pair<m_per_sec_t, radian_t> DynamicWindow::getDynamics(size_t rowIdx, size_t colIdx) const {
    return { this->wMin_speed + this->wRes_speed * rowIdx, this->wMin_wheelAngle + this->wRes_wheelAngle * colIdx };
}

} // namespace bcr