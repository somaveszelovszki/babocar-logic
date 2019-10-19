#include <local-planner/velocity_obstacle.hpp>

#include <gtest/gtest.h>

using namespace bcr;

TEST(velocity_obstacle, getDistanceToCollision_0_solutions) {
    DynamicObject obj1;
    obj1.radius = meter_t(0.5);
    obj1.odom.pose = { { meter_t(1.0), meter_t(2.0) }, degree_t(0) };
    obj1.odom.twist.speed = { m_per_sec_t(1.0), m_per_sec_t(0.0) };
    obj1.odom.twist.ang_vel = rad_per_sec_t(0.0);

    DynamicObject obj2;
    obj2.radius = meter_t(1.0);
    obj2.odom.pose = { { meter_t(4.0), meter_t(4.0) }, degree_t(180) };
    obj2.odom.twist.speed = { m_per_sec_t(-2.0), m_per_sec_t(0.0) };
    obj2.odom.twist.ang_vel = rad_per_sec_t(0.0);

    EXPECT_EQ(std::numeric_limits<float64_t>::infinity(), bcr::getDistanceToCollision(obj1, obj2).get());
}

TEST(velocity_obstacle, getDistanceToCollision_1_solution) {
    DynamicObject obj1;
    obj1.radius = meter_t(0.5);
    obj1.odom.pose = { { meter_t(1.0), meter_t(2.0) }, degree_t(0) };
    obj1.odom.twist.speed = { m_per_sec_t(1.0), m_per_sec_t(0.0) };
    obj1.odom.twist.ang_vel = rad_per_sec_t(0.0);

    DynamicObject obj2;
    obj2.radius = meter_t(1.0);
    obj2.odom.pose = { { meter_t(4.0), meter_t(3.5) }, degree_t(180) };
    obj2.odom.twist.speed = { m_per_sec_t(-2.0), m_per_sec_t(0.0) };
    obj2.odom.twist.ang_vel = rad_per_sec_t(0.0);

    EXPECT_NEAR(1.0, bcr::getDistanceToCollision(obj1, obj2).get(), 0.0001);
}

TEST(velocity_obstacle, getDistanceToCollision_2_solutions_1) {
    DynamicObject obj1;
    obj1.radius = meter_t(0.5);
    obj1.odom.pose = { { meter_t(1.0), meter_t(2.0) }, degree_t(0) };
    obj1.odom.twist.speed = { m_per_sec_t(1.0), m_per_sec_t(0.0) };
    obj1.odom.twist.ang_vel = rad_per_sec_t(0.0);

    DynamicObject obj2;
    obj2.radius = meter_t(1.0);
    obj2.odom.pose = { { meter_t(5.5), meter_t(2.0) }, degree_t(180) };
    obj2.odom.twist.speed = { m_per_sec_t(-2.0), m_per_sec_t(0.0) };
    obj2.odom.twist.ang_vel = rad_per_sec_t(0.0);

    EXPECT_NEAR(1.0, bcr::getDistanceToCollision(obj1, obj2).get(), 0.0001);
}

TEST(velocity_obstacle, getDistanceToCollision_2_solutions_2) {
    DynamicObject obj1;
    obj1.radius = meter_t(1.0);
    obj1.odom.pose = { { meter_t(0.0), meter_t(0.0) }, degree_t(90) };
    obj1.odom.twist.speed = { m_per_sec_t(0.0), m_per_sec_t(1.0) };
    obj1.odom.twist.ang_vel = rad_per_sec_t(0.0);

    DynamicObject obj2;
    obj2.radius = meter_t(1.0);
    obj2.odom.pose = { { meter_t(5.0), meter_t(5.0) }, degree_t(180) };
    obj2.odom.twist.speed = { m_per_sec_t(-1.0), m_per_sec_t(0.0) };
    obj2.odom.twist.ang_vel = rad_per_sec_t(0.0);

    EXPECT_NEAR(5.0 - std::sqrt(2), bcr::getDistanceToCollision(obj1, obj2).get(), 0.0001);
}

TEST(velocity_obstacle, getDistanceToCollision_iterative_straight) {
    DynamicObject obj1;
    obj1.radius = meter_t(1.0);
    obj1.odom.pose = { { meter_t(0.0), meter_t(0.0) }, degree_t(90) };
    obj1.odom.twist.speed = { m_per_sec_t(0.0), m_per_sec_t(1.0) };
    obj1.odom.twist.ang_vel = rad_per_sec_t(0.0);

    DynamicObject obj2;
    obj2.radius = meter_t(1.0);
    obj2.odom.pose = { { meter_t(5.0), meter_t(5.0) }, degree_t(180) };
    obj2.odom.twist.speed = { m_per_sec_t(-1.0), m_per_sec_t(0.0) };
    obj2.odom.twist.ang_vel = rad_per_sec_t(0.0);

    EXPECT_NEAR(5.0 - std::sqrt(2), bcr::getDistanceToCollision_iterative(obj1, obj2, meter_t(5), millisecond_t(50)).get(), 0.1);
}

TEST(velocity_obstacle, getDistanceToCollision_iterative_curve) {

    const m_per_sec_t speed(1.0);
    const meter_t collisionDistance = meter_t(5) * M_PI / 2;
    const millisecond_t collisionTime = collisionDistance / speed;

    DynamicObject obj1;
    obj1.radius = meter_t(1.0);
    obj1.odom.pose = { { meter_t(0.0), meter_t(0.0) }, degree_t(90) };
    obj1.odom.twist.speed = { m_per_sec_t(0.0), m_per_sec_t(1.0) };
    obj1.odom.twist.ang_vel = degree_t(-90.0) / collisionTime;

    DynamicObject obj2;
    obj2.radius = meter_t(1.0);
    obj2.odom.pose = { { meter_t(7.0), meter_t(5.0) }, degree_t(0) };
    obj2.odom.twist.speed = { m_per_sec_t(0.0), m_per_sec_t(0.0) };
    obj2.odom.twist.ang_vel = rad_per_sec_t(0.0);

    EXPECT_NEAR(collisionDistance.get(), bcr::getDistanceToCollision_iterative(obj1, obj2, meter_t(10), millisecond_t(50)).get(), 0.1);
}

TEST(velocity_obstacle, getDistanceToCollision_iterative_curve_no_collision) {

    DynamicObject obj1;
    obj1.radius = meter_t(1.0);
    obj1.odom.pose = { { meter_t(0.0), meter_t(0.0) }, degree_t(90) };
    obj1.odom.twist.speed = { m_per_sec_t(0.0), m_per_sec_t(1.0) };
    obj1.odom.twist.ang_vel = rad_per_sec_t(0.0);

    DynamicObject obj2;
    obj2.radius = meter_t(1.0);
    obj2.odom.pose = { { meter_t(7.0), meter_t(5.0) }, degree_t(0) };
    obj2.odom.twist.speed = { m_per_sec_t(0.0), m_per_sec_t(0.0) };
    obj2.odom.twist.ang_vel = rad_per_sec_t(0.0);

    EXPECT_EQ(std::numeric_limits<float64_t>::infinity(), bcr::getDistanceToCollision_iterative(obj1, obj2, meter_t(10), millisecond_t(50)).get());
}