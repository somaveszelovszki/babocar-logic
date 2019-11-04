#include <local-planner/collision.hpp>

#include <gtest/gtest.h>

using namespace bcr;

TEST(velocity_obstacle, getTimeToFirstCollision_iterative_1_static_obj_straight) {

    DynamicObject obj1(meter_t(1), {
        { { meter_t(0.0), meter_t(0.0) }, degree_t(90) }, // pose
        { { m_per_sec_t(0.0), m_per_sec_t(1.0) }, rad_per_sec_t(0.0) } // twist
    });

    StaticObject obj2;
    obj2.radius = meter_t(1.0);
    obj2.pos = { meter_t(0.0), meter_t(5.0) };

    EXPECT_NEAR(3.0, static_cast<second_t>(bcr::getTimeToFirstCollision_iterative(obj1, { obj2 }, {}, second_t(5), millisecond_t(50))).get(), 0.1);
}

TEST(velocity_obstacle, getTimeToFirstCollision_iterative_1_static_obj_curve) {

    const m_per_sec_t speed(1.0);
    const meter_t collisionDistance = meter_t(5) * M_PI / 2;
    const millisecond_t collisionTime = collisionDistance / speed;

    DynamicObject obj1(meter_t(1), {
        { { meter_t(0.0), meter_t(0.0) }, degree_t(90) }, // pose
        { { m_per_sec_t(0.0), m_per_sec_t(1.0) }, degree_t(-90.0) / collisionTime } // twist
    });

    StaticObject obj2;
    obj2.radius = meter_t(1.0);
    obj2.pos = { meter_t(7.0), meter_t(5.0) };

    EXPECT_NEAR(collisionTime.get(), bcr::getTimeToFirstCollision_iterative(obj1, { obj2 }, {}, second_t(10), millisecond_t(50)).get(), 50.0);
}

TEST(velocity_obstacle, getTimeToFirstCollision_iterative_1_dynamic_obj_straight) {

    const millisecond_t timeInterval = second_t(10);
    const millisecond_t step = millisecond_t(50);

    DynamicObject obj1(meter_t(1), {
        { { meter_t(0.0), meter_t(0.0) }, degree_t(90) }, // pose
        { { m_per_sec_t(0.0), m_per_sec_t(1.0) }, rad_per_sec_t(0.0) } // twist
    });

    DynamicObject obj2(meter_t(1), {
        { { meter_t(5.0), meter_t(5.0) }, degree_t(180) }, // pose
        { { m_per_sec_t(-1.0), m_per_sec_t(0.0) }, rad_per_sec_t(0.0) } // twist
    });
    const ObjectTrajectory traj2 = getTrajectory(obj2, timeInterval, step);

    EXPECT_NEAR(5.0 - sqrt(2), static_cast<second_t>(bcr::getTimeToFirstCollision_iterative(obj1, {}, { traj2 }, timeInterval, step)).get(), 0.1);
}

TEST(velocity_obstacle, getTimeToFirstCollision_iterative_1_static_obj_curve_no_collision) {

    DynamicObject obj1(meter_t(1), {
        { { meter_t(0.0), meter_t(0.0) }, degree_t(90) }, // pose
        { { m_per_sec_t(0.0), m_per_sec_t(1.0) }, deg_per_sec_t(20.0) } // twist
    });

    StaticObject obj2;
    obj2.radius = meter_t(1.0);
    obj2.pos = { meter_t(7.0), meter_t(5.0) };

    EXPECT_EQ(10000, bcr::getTimeToFirstCollision_iterative(obj1, { obj2 }, {}, second_t(10), millisecond_t(50)).get());
}

TEST(velocity_obstacle, getTimeToFirstCollision_iterative) {

    const m_per_sec_t speed(1.0);
    const meter_t collisionDistance = meter_t(5) * M_PI / 2;
    const millisecond_t collisionTime = collisionDistance / speed;

    const millisecond_t timeInterval = second_t(10);
    const millisecond_t step = millisecond_t(50);

    DynamicObject obj1(meter_t(1), {
        { { meter_t(0.0), meter_t(0.0) }, degree_t(90) }, // pose
        { { m_per_sec_t(0.0), m_per_sec_t(1.0) }, degree_t(-90.0) / collisionTime } // twist
    });

    StaticObject obj2;
    obj2.radius = meter_t(1.0);
    obj2.pos = { meter_t(7.0), meter_t(5.0) };

    DynamicObject obj3(meter_t(1.5), {
        { { meter_t(0.0), meter_t(5.0) }, degree_t(0) }, // pose
        { { m_per_sec_t(0.0), m_per_sec_t(1.0) }, deg_per_sec_t(10.0) } // twist
    });
    const ObjectTrajectory traj3 = getTrajectory(obj3, timeInterval, step);

    DynamicObject obj4(meter_t(0.5), {
        { { meter_t(3.0), meter_t(0.0) }, degree_t(0) }, // pose
        { { m_per_sec_t(0.5), m_per_sec_t(0.0) }, deg_per_sec_t(10.0) } // twist
    });
    const ObjectTrajectory traj4 = getTrajectory(obj4, timeInterval, step);

    EXPECT_NEAR(collisionTime.get(), bcr::getTimeToFirstCollision_iterative(obj1, { obj2 }, { traj3, traj4 }, timeInterval, step).get(), 50.0);
}

TEST(velocity_obstacle, getTimeToFirstCollision_iterative_1_dynamic_obj_curve_multiple_radiuses) {

    const m_per_sec_t speed(1.0);
    const meter_t collisionDistance = meter_t(5) * M_PI / 2;
    const millisecond_t collisionTime = collisionDistance / speed;
    const meter_t frontRearCircleDiff(1);

    const millisecond_t timeInterval = second_t(10);
    const millisecond_t step = millisecond_t(50);

    DynamicObject obj1({
        { meter_t(0), meter_t(1) },
        { frontRearCircleDiff, meter_t(1) }
    }, {
        { { meter_t(0.0), meter_t(0.0) }, degree_t(90) }, // pose
        { { m_per_sec_t(0.0), m_per_sec_t(1.0) }, degree_t(-90.0) / collisionTime } // twist
    });

    DynamicObject obj2(meter_t(1), {
        { { meter_t(7.0) + frontRearCircleDiff + collisionTime * m_per_sec_t(1.0), meter_t(5.0) }, degree_t(180) }, // pose
        { { m_per_sec_t(-1.0), m_per_sec_t(0.0) }, rad_per_sec_t(0) } // twist
    });
    const ObjectTrajectory traj2 = getTrajectory(obj2, timeInterval, step);

    EXPECT_NEAR(collisionTime.get(), bcr::getTimeToFirstCollision_iterative(obj1, {}, { traj2 }, timeInterval, step).get(), 50.0);
}