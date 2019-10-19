#include <local-planner/velocity_obstacle.hpp>

#include <gtest/gtest.h>

using namespace bcr;

TEST(velocity_obstacle, getDistanceToCollision_0_solutions) {
    DynamicObject obj1;
    obj1.radius = meter_t(0.5);
    obj1.odom.pose = { { meter_t(1.0), meter_t(2.0) }, degree_t(0) };
    obj1.odom.twist = { m_per_sec_t(1.0), m_per_sec_t(0.0), rad_per_sec_t(0.0) };

    DynamicObject obj2;
    obj2.radius = meter_t(1.0);
    obj2.odom.pose = { { meter_t(4.0), meter_t(4.0) }, degree_t(180) };
    obj2.odom.twist = { m_per_sec_t(-2.0), m_per_sec_t(0.0), rad_per_sec_t(0.0) };

    EXPECT_EQ(std::numeric_limits<float64_t>::infinity(), bcr::getDistanceToCollision(obj1, obj2).get());
}

TEST(velocity_obstacle, getDistanceToCollision_1_solution) {
    DynamicObject obj1;
    obj1.radius = meter_t(0.5);
    obj1.odom.pose = { { meter_t(1.0), meter_t(2.0) }, degree_t(0) };
    obj1.odom.twist = { m_per_sec_t(1.0), m_per_sec_t(0.0), rad_per_sec_t(0.0) };

    DynamicObject obj2;
    obj2.radius = meter_t(1.0);
    obj2.odom.pose = { { meter_t(4.0), meter_t(3.5) }, degree_t(180) };
    obj2.odom.twist = { m_per_sec_t(-2.0), m_per_sec_t(0.0), rad_per_sec_t(0.0) };

    EXPECT_NEAR(1.0, bcr::getDistanceToCollision(obj1, obj2).get(), 0.0001);
}

TEST(velocity_obstacle, getDistanceToCollision_2_solutions_1) {
    DynamicObject obj1;
    obj1.radius = meter_t(0.5);
    obj1.odom.pose = { { meter_t(1.0), meter_t(2.0) }, degree_t(0) };
    obj1.odom.twist = { m_per_sec_t(1.0), m_per_sec_t(0.0), rad_per_sec_t(0.0) };

    DynamicObject obj2;
    obj2.radius = meter_t(1.0);
    obj2.odom.pose = { { meter_t(5.5), meter_t(2.0) }, degree_t(180) };
    obj2.odom.twist = { m_per_sec_t(-2.0), m_per_sec_t(0.0), rad_per_sec_t(0.0) };

    EXPECT_NEAR(1.0, bcr::getDistanceToCollision(obj1, obj2).get(), 0.0001);
}

TEST(velocity_obstacle, getDistanceToCollision_2_solutions_2) {
    DynamicObject obj1;
    obj1.radius = meter_t(1.0);
    obj1.odom.pose = { { meter_t(0.0), meter_t(0.0) }, degree_t(90) };
    obj1.odom.twist = { m_per_sec_t(0.0), m_per_sec_t(1.0), rad_per_sec_t(0.0) };

    DynamicObject obj2;
    obj2.radius = meter_t(1.0);
    obj2.odom.pose = { { meter_t(5.0), meter_t(5.0) }, degree_t(180) };
    obj2.odom.twist = { m_per_sec_t(-1.0), m_per_sec_t(0.0), rad_per_sec_t(0.0) };

    EXPECT_NEAR(5.0 - std::sqrt(2), bcr::getDistanceToCollision(obj1, obj2).get(), 0.0001);
}