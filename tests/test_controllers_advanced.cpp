#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>

#include <pidpp/experimental/cascaded_pid_controller.hpp>
#include <pidpp/experimental/gain_scheduled_pid_controller.hpp>
#include <pidpp/pidpp.hpp>

using Catch::Matchers::WithinAbs;

namespace {

pidpp::CascadedPidConfig<double> servo_config() {
    pidpp::CascadedPidConfig<double> config{};
    config.outer = {.kp = 5.0, .output_min = -100.0, .output_max = 100.0};
    config.inner = {.kp = 5.0,
                    .ki = 50.0,
                    .output_min = -12.0,
                    .output_max = 12.0,
                    .integral_min = -10.0,
                    .integral_max = 10.0,
                    .anti_windup = pidpp::AntiWindupStrategy::Clamping};
    return config;
}

} // namespace

TEST_CASE("cascade feeds the outer output into the inner setpoint") {
    auto config = servo_config();
    config.inner.ki = 0.0;
    REQUIRE(config.validate());
    pidpp::CascadedPidController<double> cascade{config};

    const double command = cascade.update(0.1, 0.0, 0.0, 0.001);

    REQUIRE_THAT(cascade.inner_setpoint(), WithinAbs(0.5, 1.0e-12));
    REQUIRE_THAT(command, WithinAbs(2.5, 1.0e-12));
}

TEST_CASE("cascade outer loop honors decimation") {
    auto config = servo_config();
    config.outer_decimation = 4;
    pidpp::CascadedPidController<double> cascade{config};

    static_cast<void>(cascade.update(1.0, 0.0, 0.0, 0.001));
    const double first_setpoint = cascade.inner_setpoint();
    for (int i = 0; i < 3; ++i) {
        static_cast<void>(cascade.update(2.0, 0.0, 0.0, 0.001));
        REQUIRE_THAT(cascade.inner_setpoint(), WithinAbs(first_setpoint, 1.0e-12));
    }
    static_cast<void>(cascade.update(2.0, 0.0, 0.0, 0.001));

    REQUIRE_THAT(cascade.inner_setpoint(), WithinAbs(10.0, 1.0e-12));
}

TEST_CASE("cascade rejects zero decimation") {
    auto config = servo_config();
    config.outer_decimation = 0;

    REQUIRE(config.validate().code() == pidpp::ErrorCode::InvalidConfiguration);
}

TEST_CASE("cascaded servo positions a DC motor") {
    auto config = servo_config();
    REQUIRE(config.validate());
    pidpp::CascadedPidController<double> cascade{config};
    pidpp::DcMotor<double> motor{2.0, 0.01, 0.1, 0.1, 0.01, 0.001};

    double voltage = 0.0;
    for (int i = 0; i < 3000; ++i) {
        motor.update(voltage, 0.0, 0.001);
        voltage = cascade.update(1.0, motor.position(), motor.angular_velocity(), 0.001);
    }

    REQUIRE_THAT(motor.position(), WithinAbs(1.0, 1.0e-3));
    cascade.reset();
    REQUIRE_THAT(cascade.inner_setpoint(), WithinAbs(0.0, 1.0e-12));
}

TEST_CASE("experimental cascade alias still compiles") {
    pidpp::experimental::CascadedPidControllerConfig<double> config = servo_config();
    pidpp::experimental::CascadedPidController<double> cascade{config};

    REQUIRE(cascade.inner_setpoint() == 0.0);
}

TEST_CASE("gain schedule interpolates and holds the ends") {
    pidpp::PidConfig<double> base{.output_min = -100.0, .output_max = 100.0};
    pidpp::GainScheduledPidController<double, 4> controller{base};
    const std::array<pidpp::GainSchedulePoint<double>, 2> points{{
        {0.0, {1.0, 0.0, 0.0}},
        {10.0, {3.0, 2.0, 1.0}},
    }};

    REQUIRE(controller.set_schedule(points));
    REQUIRE(controller.size() == 2);

    const auto mid = controller.gains_at(5.0);
    REQUIRE_THAT(mid.kp, WithinAbs(2.0, 1.0e-12));
    REQUIRE_THAT(mid.ki, WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(mid.kd, WithinAbs(0.5, 1.0e-12));
    REQUIRE_THAT(controller.gains_at(-3.0).kp, WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(controller.gains_at(50.0).kp, WithinAbs(3.0, 1.0e-12));

    const double output = controller.update(5.0, 1.0, 0.0, 0.01);
    REQUIRE_THAT(controller.controller().config().kp, WithinAbs(2.0, 1.0e-12));
    REQUIRE(output > 0.0);
}

TEST_CASE("gain schedule rejects unsorted, oversize, and invalid tables") {
    pidpp::GainScheduledPidController<double, 2> controller{pidpp::PidConfig<double>{}};

    const std::array<pidpp::GainSchedulePoint<double>, 2> unsorted{{
        {1.0, {1.0, 0.0, 0.0}},
        {1.0, {2.0, 0.0, 0.0}},
    }};
    REQUIRE_FALSE(controller.set_schedule(unsorted));

    const std::array<pidpp::GainSchedulePoint<double>, 3> oversize{};
    REQUIRE_FALSE(controller.set_schedule(oversize));

    const std::array<pidpp::GainSchedulePoint<double>, 1> negative{{{0.0, {-1.0, 0.0, 0.0}}}};
    REQUIRE(controller.set_schedule(negative).code() == pidpp::ErrorCode::InvalidGain);

    REQUIRE(controller.size() == 0);
}

TEST_CASE("experimental gain schedule alias still compiles") {
    pidpp::experimental::GainScheduledPidController<double, 2> controller{
        pidpp::PidConfig<double>{}};
    const pidpp::experimental::GainSchedulePoint<double> point{0.0, {1.0, 0.0, 0.0}};

    REQUIRE(controller.set_schedule(std::span{&point, 1}));
}
