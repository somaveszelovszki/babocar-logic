#include <local-planner/trajectory.hpp>

#include <gtest/gtest.h>

using namespace bcr;

TEST(trajectory, getTrajectoryWheelAngle_straight) {
    static constexpr meter_t carFrontRearWheelAxisDist = centimeter_t(65);

    const Point2m carPos = { meter_t(0), meter_t(0) };
    const Point2m destPos = { meter_t(0), meter_t(10) };
    const radian_t ori = degree_t(90);

    EXPECT_NEAR(0.0, getTrajectoryWheelAngle(carFrontRearWheelAxisDist, carPos, ori, destPos).get(), 0.1);
}

TEST(trajectory, getTrajectoryWheelAngle_left) {
    static constexpr meter_t carFrontRearWheelAxisDist = centimeter_t(65);

    const Point2m carPos = { meter_t(0), meter_t(0) };
    const Point2m destPos = { meter_t(-5), meter_t(-5) };
    const radian_t ori = degree_t(-90);

    EXPECT_NEAR((bcr::atan2(-carFrontRearWheelAxisDist.get(), 5.0)).get(), getTrajectoryWheelAngle(carFrontRearWheelAxisDist, carPos, ori, destPos).get(), 0.1);
}