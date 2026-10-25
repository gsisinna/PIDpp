#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <numbers>

#include <pidpp/pidpp.hpp>

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

TEST_CASE("first-order plant moves toward steady state") {
    pidpp::FirstOrderPlant<double> plant{2.0, 1.0};

    const double y0 = plant.update(1.0, 0.1);
    const double y1 = plant.update(1.0, 0.1);

    REQUIRE(y0 > 0.0);
    REQUIRE(y1 > y0);
    REQUIRE(y1 < 2.0);
}

TEST_CASE("DC motor speed increases under positive voltage") {
    pidpp::DcMotor<double> motor{2.0, 0.01, 0.1, 0.1, 0.01, 0.001};

    const double w0 = motor.update(6.0, 0.0, 0.001);
    double w1 = w0;
    for (int i = 0; i < 100; ++i) {
        w1 = motor.update(6.0, 0.0, 0.001);
    }

    REQUIRE(w0 >= 0.0);
    REQUIRE(w1 > w0);
    REQUIRE(motor.current() > 0.0);
}

TEST_CASE("DC motor stays stable with a coarse time step") {
    // dt = 20 ms is 4x the electrical time constant L/R = 5 ms. Without
    // internal sub-stepping explicit Euler diverges here.
    pidpp::DcMotor<double> motor{2.0, 0.01, 0.1, 0.1, 0.01, 0.001};

    for (int i = 0; i < 1000; ++i) {
        motor.update(6.0, 0.0, 0.02);
    }

    REQUIRE(std::isfinite(motor.angular_velocity()));
    REQUIRE_THAT(motor.angular_velocity(), WithinRel(motor.steady_state_speed(6.0), 1.0e-3));
}

TEST_CASE("mass spring damper responds to force") {
    pidpp::MassSpringDamper<double> plant{1.0, 0.2, 2.0};

    const double p0 = plant.update(1.0, 0.01);
    const double p1 = plant.update(1.0, 0.01);

    REQUIRE(p0 > 0.0);
    REQUIRE(p1 > p0);
}

TEST_CASE("single link arm falls under gravity and holds with compensation") {
    pidpp::SingleLinkArm<double> arm{2.0, 0.5, 0.1};

    REQUIRE_THAT(arm.gravity_torque(0.0), WithinAbs(2.0 * 0.25 * 9.80665, 1.0e-9));
    REQUIRE_THAT(arm.gravity_torque(std::numbers::pi / 2.0), WithinAbs(0.0, 1.0e-9));

    arm.update(0.0, 0.01);
    REQUIRE(arm.angle() < 0.0);

    arm.reset(0.3);
    for (int i = 0; i < 1000; ++i) {
        arm.update(arm.gravity_torque(arm.angle()), 0.001);
    }
    REQUIRE_THAT(arm.angle(), WithinAbs(0.3, 1.0e-9));

    const double inertia = arm.total_inertia();
    arm.set_payload_mass(1.0);
    REQUIRE(arm.total_inertia() > inertia);
}

TEST_CASE("differential drive drives straight and turns") {
    pidpp::DifferentialDrive<double> robot{0.0, 0.0};

    for (int i = 0; i < 100; ++i) {
        robot.update(1.0, 0.0, 0.01);
    }
    REQUIRE_THAT(robot.pose().x, WithinAbs(1.0, 1.0e-9));
    REQUIRE_THAT(robot.pose().y, WithinAbs(0.0, 1.0e-9));

    for (int i = 0; i < 100; ++i) {
        robot.update(0.0, std::numbers::pi, 0.01);
    }
    REQUIRE_THAT(std::abs(robot.pose().heading), WithinAbs(std::numbers::pi, 1.0e-9));
}

TEST_CASE("vertical thrust plant hovers at m*g and rests on the ground") {
    pidpp::VerticalThrustPlant<double> drone{1.5, 0.2, 0.0};

    drone.update(0.0, 0.01);
    REQUIRE(drone.altitude() == 0.0);

    drone.reset(5.0);
    for (int i = 0; i < 1000; ++i) {
        drone.update(drone.hover_thrust(), 0.001);
    }
    REQUIRE_THAT(drone.altitude(), WithinAbs(5.0, 1.0e-9));
}
