#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <atomic>
#include <cstdlib>
#include <limits>
#include <new>
#include <numbers>
#include <type_traits>

#include <pidpp/pidpp.hpp>

using Catch::Matchers::WithinAbs;

namespace {

std::atomic<bool> track_allocations{false};
std::atomic<int> allocation_count{0};

} // namespace

void* operator new(std::size_t size) {
    if (track_allocations.load(std::memory_order_relaxed)) {
        allocation_count.fetch_add(1, std::memory_order_relaxed);
    }
    if (void* ptr = std::malloc(size)) {
        return ptr;
    }
    throw std::bad_alloc{};
}

void operator delete(void* ptr) noexcept {
    std::free(ptr);
}

void operator delete(void* ptr, std::size_t) noexcept {
    std::free(ptr);
}

namespace {

pidpp::PidConfig<double> base_config() {
    pidpp::PidConfig<double> config{};
    config.output_min = -10.0;
    config.output_max = 10.0;
    config.integral_min = -5.0;
    config.integral_max = 5.0;
    return config;
}

} // namespace

TEST_CASE("proportional control returns gain scaled error") {
    auto config = base_config();
    config.kp = 2.0;
    pidpp::PidController<double> controller{config};
    static_assert(noexcept(controller.update(0.0, 0.0, 0.001)));

    const double output = controller.update(3.0, 1.0, 0.01);

    REQUIRE_THAT(output, WithinAbs(4.0, 1.0e-12));
    REQUIRE_THAT(controller.state().proportional_term, WithinAbs(4.0, 1.0e-12));
}

TEST_CASE("integral term accumulates over valid updates") {
    auto config = base_config();
    config.ki = 2.0;
    pidpp::PidController<double> controller{config};

    static_cast<void>(controller.update(1.0, 0.0, 0.5));
    const double output = controller.update(1.0, 0.0, 0.5);

    REQUIRE_THAT(output, WithinAbs(2.0, 1.0e-12));
    REQUIRE_THAT(controller.state().integral, WithinAbs(1.0, 1.0e-12));
}

TEST_CASE("derivative on error uses error slope after first sample") {
    auto config = base_config();
    config.kd = 0.5;
    config.derivative_mode = pidpp::DerivativeMode::OnError;
    pidpp::PidController<double> controller{config};

    static_cast<void>(controller.update(1.0, 0.0, 0.1));
    const double output = controller.update(2.0, 0.0, 0.1);

    REQUIRE_THAT(output, WithinAbs(5.0, 1.0e-12));
    REQUIRE_THAT(controller.state().derivative, WithinAbs(10.0, 1.0e-12));
}

TEST_CASE("derivative on measurement avoids setpoint kick") {
    auto config = base_config();
    config.kd = 1.0;
    config.derivative_mode = pidpp::DerivativeMode::OnMeasurement;
    pidpp::PidController<double> controller{config};

    static_cast<void>(controller.update(1.0, 0.0, 0.1));
    const double output = controller.update(2.0, 0.0, 0.1);

    REQUIRE_THAT(output, WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(controller.state().derivative, WithinAbs(0.0, 1.0e-12));
}

TEST_CASE("output saturation clamps command") {
    auto config = base_config();
    config.kp = 100.0;
    config.output_min = -2.0;
    config.output_max = 2.0;
    pidpp::PidController<double> controller{config};

    const double output = controller.update(1.0, 0.0, 0.1);

    REQUIRE_THAT(output, WithinAbs(2.0, 1.0e-12));
    REQUIRE(controller.state().saturated);
}

TEST_CASE("conditional integration blocks deeper saturation") {
    auto config = base_config();
    config.ki = 10.0;
    config.output_min = -1.0;
    config.output_max = 1.0;
    config.anti_windup = pidpp::AntiWindupStrategy::ConditionalIntegration;
    pidpp::PidController<double> controller{config};

    for (int i = 0; i < 10; ++i) {
        static_cast<void>(controller.update(10.0, 0.0, 0.1));
    }

    REQUIRE_THAT(controller.state().integral, WithinAbs(0.0, 1.0e-12));
}

TEST_CASE("anti-windup none integrates while output is saturated") {
    auto config = base_config();
    config.ki = 10.0;
    config.output_min = -1.0;
    config.output_max = 1.0;
    config.anti_windup = pidpp::AntiWindupStrategy::None;
    pidpp::PidController<double> controller{config};

    static_cast<void>(controller.update(10.0, 0.0, 0.1));

    REQUIRE(controller.state().integral > 0.0);
    REQUIRE_THAT(controller.state().output, WithinAbs(1.0, 1.0e-12));
}

TEST_CASE("anti-windup clamping keeps integral inside configured limits") {
    auto config = base_config();
    config.ki = 10.0;
    config.integral_min = -0.25;
    config.integral_max = 0.25;
    config.anti_windup = pidpp::AntiWindupStrategy::Clamping;
    pidpp::PidController<double> controller{config};

    for (int i = 0; i < 10; ++i) {
        static_cast<void>(controller.update(10.0, 0.0, 0.1));
    }

    REQUIRE_THAT(controller.state().integral, WithinAbs(0.25, 1.0e-12));
}

TEST_CASE("back calculation reduces integral pressure while saturated") {
    auto config = base_config();
    config.ki = 5.0;
    config.output_min = -1.0;
    config.output_max = 1.0;
    config.anti_windup = pidpp::AntiWindupStrategy::BackCalculation;
    config.back_calculation_gain = 1.0;
    pidpp::PidController<double> controller{config};

    static_cast<void>(controller.update(10.0, 0.0, 0.1));

    REQUIRE(controller.state().integral < 1.0);
}

TEST_CASE("runtime gain update changes controller response") {
    auto config = base_config();
    config.kp = 1.0;
    pidpp::PidController<double> controller{config};

    const double first = controller.update(2.0, 1.0, 0.01);
    controller.set_gains(3.0, 0.0, 0.0);
    const double second = controller.update(2.0, 1.0, 0.01);

    REQUIRE_THAT(first, WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(second, WithinAbs(3.0, 1.0e-12));
}

TEST_CASE("reset clears state") {
    auto config = base_config();
    config.kp = 1.0;
    config.ki = 1.0;
    pidpp::PidController<double> controller{config};

    static_cast<void>(controller.update(1.0, 0.0, 0.1));
    controller.reset();

    REQUIRE_THAT(controller.state().integral, WithinAbs(0.0, 1.0e-12));
    REQUIRE_FALSE(controller.state().initialized);
}

TEST_CASE("invalid dt returns previous output safely") {
    auto config = base_config();
    config.kp = 1.0;
    pidpp::PidController<double> controller{config};

    const double first = controller.update(2.0, 1.0, 0.1);
    const double second = controller.update(100.0, 0.0, 0.0);

    REQUIRE_THAT(second, WithinAbs(first, 1.0e-12));
    REQUIRE_FALSE(controller.state().last_dt_valid);
}

TEST_CASE("feedforward can be added without changing PID gains") {
    auto config = base_config();
    config.feedforward.enabled = true;
    config.feedforward.velocity_gain = 2.0;
    config.feedforward.acceleration_gain = 3.0;
    config.feedforward.static_gain = 1.0;
    pidpp::PidController<double> controller{config};

    const double output = controller.update(0.0, 0.0, 0.01, 4.0, 5.0);

    REQUIRE_THAT(output, WithinAbs(10.0, 1.0e-12));
}

TEST_CASE("update performs no dynamic allocation") {
    auto config = base_config();
    config.kp = 1.2;
    config.ki = 0.4;
    config.kd = 0.05;
    config.derivative_filter_tau = 0.02;
    config.anti_windup = pidpp::AntiWindupStrategy::ConditionalIntegration;
    pidpp::PidController<double> controller{config};

    allocation_count.store(0, std::memory_order_relaxed);
    track_allocations.store(true, std::memory_order_relaxed);
    for (int i = 0; i < 1000; ++i) {
        static_cast<void>(controller.update(10.0, 8.5, 0.001));
    }
    track_allocations.store(false, std::memory_order_relaxed);

    REQUIRE(allocation_count.load(std::memory_order_relaxed) == 0);
}

TEST_CASE("PID state types are lightweight") {
    STATIC_REQUIRE(std::is_trivially_copyable_v<pidpp::PidState<double>>);
}

TEST_CASE("error deadband ignores small errors without a step at the edge") {
    auto config = base_config();
    config.kp = 1.0;
    config.error_deadband = 0.5;
    pidpp::PidController<double> controller{config};

    REQUIRE_THAT(controller.update(0.3, 0.0, 0.01), WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(controller.update(-0.5, 0.0, 0.01), WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(controller.update(2.0, 0.0, 0.01), WithinAbs(1.5, 1.0e-12));
    REQUIRE_THAT(controller.update(-2.0, 0.0, 0.01), WithinAbs(-1.5, 1.0e-12));
}

TEST_CASE("continuous input takes the shortest way around the circle") {
    auto config = base_config();
    config.kp = 1.0;
    config.continuous_input = true;
    pidpp::PidController<double> controller{config};

    // From +170 deg to -170 deg is a +20 deg turn, not a -340 deg one.
    const double setpoint = pidpp::deg_to_rad(-170.0);
    const double measurement = pidpp::deg_to_rad(170.0);
    const double output = controller.update(setpoint, measurement, 0.01);

    REQUIRE_THAT(output, WithinAbs(pidpp::deg_to_rad(20.0), 1.0e-9));
}

TEST_CASE("continuous input derivative on measurement ignores wrap-around") {
    auto config = base_config();
    config.kd = 1.0;
    config.derivative_mode = pidpp::DerivativeMode::OnMeasurement;
    config.continuous_input = true;
    pidpp::PidController<double> controller{config};

    static_cast<void>(controller.update(0.0, 3.1, 0.1));
    static_cast<void>(controller.update(0.0, -3.1, 0.1));

    // The measurement moved forward across the seam by 2*pi - 6.2 rad.
    const double expected = -((2.0 * std::numbers::pi) - 6.2) / 0.1;
    REQUIRE_THAT(controller.state().derivative, WithinAbs(expected, 1.0e-9));
}

TEST_CASE("output rate limit bounds the command slew") {
    auto config = base_config();
    config.kp = 100.0;
    config.output_rate_limit = 10.0;
    pidpp::PidController<double> controller{config};

    const double first = controller.update(1.0, 0.0, 0.01);
    const double second = controller.update(1.0, 0.0, 0.01);

    REQUIRE_THAT(first, WithinAbs(0.1, 1.0e-12));
    REQUIRE_THAT(second, WithinAbs(0.2, 1.0e-12));
    REQUIRE(controller.state().saturated);
}

TEST_CASE("output rate limit yields to reconfigured hard limits") {
    auto config = base_config();
    config.kp = 100.0;
    pidpp::PidController<double> controller{config};
    REQUIRE_THAT(controller.update(1.0, 0.0, 0.01), WithinAbs(10.0, 1.0e-12));

    config.output_max = 2.0;
    config.output_rate_limit = 1.0;
    controller.set_config(config);

    REQUIRE_THAT(controller.update(1.0, 0.0, 0.01), WithinAbs(2.0, 1.0e-12));
}

TEST_CASE("changing ki keeps the integral contribution continuous") {
    auto config = base_config();
    config.ki = 2.0;
    pidpp::PidController<double> controller{config};

    static_cast<void>(controller.update(1.0, 0.0, 0.5));
    const double before = controller.state().integral_term;
    controller.set_gains(0.0, 4.0, 0.0);

    REQUIRE_THAT(controller.state().integral_term, WithinAbs(before, 1.0e-12));
    REQUIRE_THAT(controller.state().integral, WithinAbs(0.25, 1.0e-12));
}

TEST_CASE("bumpless transfer reproduces the current actuator command") {
    auto config = base_config();
    config.kp = 2.0;
    config.ki = 1.0;
    pidpp::PidController<double> controller{config};

    controller.bumpless_transfer(3.0, 1.0, 0.5);
    const double output = controller.update(1.0, 0.5, 0.001);

    REQUIRE_THAT(output, WithinAbs(3.0, 1.0e-3));
}

TEST_CASE("dynamic clamping stops windup while proportional action saturates") {
    auto config = base_config();
    config.kp = 20.0;
    config.ki = 1.0;
    config.output_min = -1.0;
    config.output_max = 1.0;
    config.anti_windup = pidpp::AntiWindupStrategy::Clamping;
    pidpp::PidController<double> controller{config};

    for (int i = 0; i < 100; ++i) {
        static_cast<void>(controller.update(1.0, 0.0, 0.1));
    }

    // P alone already exceeds the limit, so the integral may not grow.
    REQUIRE_THAT(controller.state().integral, WithinAbs(0.0, 1.0e-12));
}

TEST_CASE("non finite feedforward input is rejected") {
    auto config = base_config();
    config.kp = 1.0;
    pidpp::PidController<double> controller{config};

    const double first = controller.update(1.0, 0.0, 0.01);
    const double second =
        controller.update(1.0, 0.0, 0.01, std::numeric_limits<double>::quiet_NaN(), 0.0);

    REQUIRE_THAT(second, WithinAbs(first, 1.0e-12));
    REQUIRE_FALSE(controller.state().last_dt_valid);
}

TEST_CASE("external feedforward is added before saturation") {
    auto config = base_config();
    config.kp = 1.0;
    config.ki = 1.0;
    config.output_max = 3.0;
    config.anti_windup = pidpp::AntiWindupStrategy::Clamping;
    pidpp::PidController<double> controller{config};

    REQUIRE_THAT(controller.update_with_feedforward(1.0, 0.0, 0.1, 1.5), WithinAbs(2.6, 1.0e-12));
    REQUIRE_THAT(controller.state().feedforward_term, WithinAbs(1.5, 1.0e-12));
    REQUIRE_THAT(controller.update_with_feedforward(1.0, 0.0, 0.1, 10.0), WithinAbs(3.0, 1.0e-12));
    REQUIRE(controller.state().saturated);
    REQUIRE_THAT(
        controller.update_with_feedforward(1.0, 0.0, 0.1, std::numeric_limits<double>::quiet_NaN()),
        WithinAbs(3.0, 1.0e-12));
    REQUIRE_FALSE(controller.state().last_dt_valid);
}

TEST_CASE("friction feedforward follows the sign of the reference velocity") {
    auto config = base_config();
    config.feedforward.enabled = true;
    config.feedforward.friction_gain = 0.5;
    pidpp::PidController<double> controller{config};

    REQUIRE_THAT(controller.update(0.0, 0.0, 0.01, 2.0, 0.0), WithinAbs(0.5, 1.0e-12));
    REQUIRE_THAT(controller.update(0.0, 0.0, 0.01, -2.0, 0.0), WithinAbs(-0.5, 1.0e-12));
    REQUIRE_THAT(controller.update(0.0, 0.0, 0.01, 0.0, 0.0), WithinAbs(0.0, 1.0e-12));
}

TEST_CASE("PI loop removes steady-state error on a first-order plant") {
    pidpp::PidConfig<double> config{.kp = 2.0,
                                    .ki = 4.0,
                                    .output_min = -10.0,
                                    .output_max = 10.0,
                                    .integral_min = -10.0,
                                    .integral_max = 10.0,
                                    .anti_windup = pidpp::AntiWindupStrategy::Clamping};
    REQUIRE(config.validate());
    pidpp::PidController<double> controller{config};
    pidpp::FirstOrderPlant<double> plant{1.0, 0.5};

    double command = 0.0;
    for (int i = 0; i < 5000; ++i) {
        command = controller.update(2.0, plant.update(command, 0.001), 0.001);
    }

    REQUIRE_THAT(plant.state(), WithinAbs(2.0, 1.0e-3));
}
