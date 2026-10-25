#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <numbers>
#include <vector>

#include <pidpp/pidpp.hpp>

using Catch::Matchers::WithinAbs;

TEST_CASE("trapezoidal profile reaches cruise velocity on long moves") {
    pidpp::TrapezoidalProfile<double> profile;
    REQUIRE(profile.plan(0.0, 10.0, {.max_velocity = 2.0, .max_acceleration = 1.0}));

    // 2 s accelerating (2 m), 3 s cruising (6 m), 2 s braking (2 m).
    REQUIRE_THAT(profile.duration(), WithinAbs(7.0, 1.0e-12));
    REQUIRE_THAT(profile.peak_velocity(), WithinAbs(2.0, 1.0e-12));

    const auto accel = profile.sample(1.0);
    REQUIRE_THAT(accel.position, WithinAbs(0.5, 1.0e-12));
    REQUIRE_THAT(accel.velocity, WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(accel.acceleration, WithinAbs(1.0, 1.0e-12));

    const auto cruise = profile.sample(3.5);
    REQUIRE_THAT(cruise.position, WithinAbs(5.0, 1.0e-12));
    REQUIRE_THAT(cruise.velocity, WithinAbs(2.0, 1.0e-12));

    const auto brake = profile.sample(6.0);
    REQUIRE_THAT(brake.position, WithinAbs(9.5, 1.0e-12));
    REQUIRE_THAT(brake.acceleration, WithinAbs(-1.0, 1.0e-12));

    const auto end = profile.sample(100.0);
    REQUIRE_THAT(end.position, WithinAbs(10.0, 1.0e-12));
    REQUIRE_THAT(end.velocity, WithinAbs(0.0, 1.0e-12));
    REQUIRE(profile.finished(7.0));
}

TEST_CASE("trapezoidal profile degrades to a triangle on short moves") {
    pidpp::TrapezoidalProfile<double> profile;
    REQUIRE(profile.plan(1.0, 0.0, {.max_velocity = 10.0, .max_acceleration = 4.0}));

    REQUIRE_THAT(profile.duration(), WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(profile.peak_velocity(), WithinAbs(2.0, 1.0e-12));
    const auto mid = profile.sample(0.5);
    REQUIRE_THAT(mid.position, WithinAbs(0.5, 1.0e-12));
    REQUIRE_THAT(mid.velocity, WithinAbs(-2.0, 1.0e-12));
}

TEST_CASE("trapezoidal profile keeps the previous plan on invalid input") {
    pidpp::TrapezoidalProfile<double> profile;
    REQUIRE(profile.plan(0.0, 1.0, {}));
    const double duration = profile.duration();

    REQUIRE_FALSE(profile.plan(0.0, 5.0, {.max_velocity = 0.0}));
    REQUIRE_THAT(profile.duration(), WithinAbs(duration, 1.0e-12));
}

TEST_CASE("slew rate limiter bounds rising and falling rates independently") {
    pidpp::SlewRateLimiter<double> limiter{1.0, 4.0, 0.0};

    REQUIRE_THAT(limiter.update(10.0, 0.5), WithinAbs(0.5, 1.0e-12));
    REQUIRE_THAT(limiter.update(10.0, 0.5), WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(limiter.update(-10.0, 0.1), WithinAbs(0.6, 1.0e-12));
    REQUIRE_THAT(limiter.update(0.55, 0.1), WithinAbs(0.55, 1.0e-12));
    REQUIRE_THAT(limiter.update(5.0, 0.0), WithinAbs(0.55, 1.0e-12));
}

TEST_CASE("angle helpers wrap to the principal range") {
    constexpr double pi = std::numbers::pi;
    REQUIRE_THAT(pidpp::wrap_angle(2.5 * pi), WithinAbs(0.5 * pi, 1.0e-12));
    REQUIRE_THAT(pidpp::wrap_angle(-0.5), WithinAbs(-0.5, 1.0e-12));
    REQUIRE_THAT(pidpp::angle_difference(-3.0, 3.0), WithinAbs((2.0 * pi) - 6.0, 1.0e-12));
    REQUIRE_THAT(pidpp::wrap_to_range(370.0, 0.0, 360.0), WithinAbs(10.0, 1.0e-12));
    REQUIRE_THAT(pidpp::wrap_to_range(-10.0, 0.0, 360.0), WithinAbs(350.0, 1.0e-12));
    REQUIRE_THAT(pidpp::rad_to_deg(pidpp::deg_to_rad(45.0)), WithinAbs(45.0, 1.0e-12));
    REQUIRE_THAT(pidpp::apply_deadband(0.2, 0.5), WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(pidpp::apply_deadband(-1.0, 0.5), WithinAbs(-0.5, 1.0e-12));
}

TEST_CASE("step metrics on a first-order rise") {
    std::vector<double> time;
    std::vector<double> output;
    pidpp::FirstOrderPlant<double> plant{1.0, 0.2};
    for (int i = 0; i <= 2000; ++i) {
        time.push_back(static_cast<double>(i) * 0.001);
        output.push_back(plant.state());
        plant.update(1.0, 0.001);
    }

    const auto metrics = pidpp::analyze_step_response<double>(time, output, 1.0);

    // 10-90 % rise time of a first-order lag is tau * ln(9) ~= 0.439 s.
    REQUIRE(metrics.rose);
    REQUIRE_THAT(metrics.rise_time, WithinAbs(0.2 * 2.1972, 5.0e-3));
    REQUIRE(metrics.settled);
    // 2 % settling of a first-order lag is tau * ln(50) ~= 0.782 s.
    REQUIRE_THAT(metrics.settling_time, WithinAbs(0.2 * 3.912, 5.0e-3));
    REQUIRE_THAT(metrics.overshoot_percent, WithinAbs(0.0, 1.0e-12));
    // IAE of a first-order step response tends to tau.
    REQUIRE_THAT(metrics.integral_absolute_error, WithinAbs(0.2, 5.0e-3));
}

TEST_CASE("step metrics handle downward steps from a non-zero start") {
    const std::array<double, 6> time{0.0, 1.0, 2.0, 3.0, 4.0, 5.0};
    const std::array<double, 6> output{10.0, 6.0, 1.0, 3.0, 2.1, 2.0};

    const auto metrics = pidpp::analyze_step_response<double>(
        time, output, pidpp::StepResponseOptions<double>{.initial_value = 10.0, .target = 2.0});

    REQUIRE_THAT(metrics.peak_value, WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(metrics.overshoot_percent, WithinAbs(12.5, 1.0e-9));
    REQUIRE(metrics.rose);
    REQUIRE(metrics.settled);
    REQUIRE_THAT(metrics.settling_time, WithinAbs(4.0, 1.0e-12));
    REQUIRE_THAT(metrics.steady_state_error, WithinAbs(0.0, 1.0e-12));
}

TEST_CASE("step metrics report unsettled responses") {
    const std::array<double, 4> time{0.0, 1.0, 2.0, 3.0};
    const std::array<double, 4> output{0.0, 0.5, 0.6, 0.7};

    const auto metrics = pidpp::analyze_step_response<double>(time, output, 1.0);

    REQUIRE_FALSE(metrics.settled);
    REQUIRE_FALSE(metrics.rose);
    REQUIRE_THAT(metrics.steady_state_error, WithinAbs(0.3, 1.0e-12));
}

TEST_CASE("step metrics reject degenerate input") {
    const std::array<double, 2> time{0.0, 1.0};
    const std::array<double, 1> output{1.0};

    REQUIRE_FALSE(pidpp::analyze_step_response<double>(time, output, 1.0).settled);
    REQUIRE_FALSE(pidpp::analyze_step_response<double>(time, time, 0.0).settled);
}

TEST_CASE("tracking metrics") {
    const std::array<double, 4> reference{0.0, 1.0, 2.0, 3.0};
    const std::array<double, 4> measured{0.0, 1.0, 1.0, 4.0};

    const auto metrics = pidpp::analyze_tracking<double>(reference, measured);

    REQUIRE_THAT(metrics.max_abs_error, WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(metrics.mean_abs_error, WithinAbs(0.5, 1.0e-12));
    REQUIRE_THAT(metrics.rms_error, WithinAbs(std::sqrt(0.5), 1.0e-12));
}
