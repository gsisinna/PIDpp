#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <atomic>
#include <cstdlib>
#include <new>
#include <type_traits>

#include <pidpp/pidpp.hpp>

using Catch::Matchers::WithinAbs;

namespace {

std::atomic<bool> track_allocations{false};
std::atomic<int> allocation_count{0};

} // namespace

void* operator new(std::size_t size)
{
    if (track_allocations.load(std::memory_order_relaxed)) {
        allocation_count.fetch_add(1, std::memory_order_relaxed);
    }
    if (void* ptr = std::malloc(size)) {
        return ptr;
    }
    throw std::bad_alloc{};
}

void operator delete(void* ptr) noexcept
{
    std::free(ptr);
}

void operator delete(void* ptr, std::size_t) noexcept
{
    std::free(ptr);
}

namespace {

pidpp::PidConfig<double> base_config()
{
    pidpp::PidConfig<double> config{};
    config.output_min = -10.0;
    config.output_max = 10.0;
    config.integral_min = -5.0;
    config.integral_max = 5.0;
    return config;
}

} // namespace

TEST_CASE("proportional control returns gain scaled error")
{
    auto config = base_config();
    config.kp = 2.0;
    pidpp::PidController<double> controller{config};
    static_assert(noexcept(controller.update(0.0, 0.0, 0.001)));

    const double output = controller.update(3.0, 1.0, 0.01);

    REQUIRE_THAT(output, WithinAbs(4.0, 1.0e-12));
    REQUIRE_THAT(controller.state().proportional_term, WithinAbs(4.0, 1.0e-12));
}

TEST_CASE("integral term accumulates over valid updates")
{
    auto config = base_config();
    config.ki = 2.0;
    pidpp::PidController<double> controller{config};

    static_cast<void>(controller.update(1.0, 0.0, 0.5));
    const double output = controller.update(1.0, 0.0, 0.5);

    REQUIRE_THAT(output, WithinAbs(2.0, 1.0e-12));
    REQUIRE_THAT(controller.state().integral, WithinAbs(1.0, 1.0e-12));
}

TEST_CASE("derivative on error uses error slope after first sample")
{
    auto config = base_config();
    config.kd = 0.5;
    config.derivative_mode = pidpp::DerivativeMode::OnError;
    pidpp::PidController<double> controller{config};

    static_cast<void>(controller.update(1.0, 0.0, 0.1));
    const double output = controller.update(2.0, 0.0, 0.1);

    REQUIRE_THAT(output, WithinAbs(5.0, 1.0e-12));
    REQUIRE_THAT(controller.state().derivative, WithinAbs(10.0, 1.0e-12));
}

TEST_CASE("derivative on measurement avoids setpoint kick")
{
    auto config = base_config();
    config.kd = 1.0;
    config.derivative_mode = pidpp::DerivativeMode::OnMeasurement;
    pidpp::PidController<double> controller{config};

    static_cast<void>(controller.update(1.0, 0.0, 0.1));
    const double output = controller.update(2.0, 0.0, 0.1);

    REQUIRE_THAT(output, WithinAbs(0.0, 1.0e-12));
    REQUIRE_THAT(controller.state().derivative, WithinAbs(0.0, 1.0e-12));
}

TEST_CASE("output saturation clamps command")
{
    auto config = base_config();
    config.kp = 100.0;
    config.output_min = -2.0;
    config.output_max = 2.0;
    pidpp::PidController<double> controller{config};

    const double output = controller.update(1.0, 0.0, 0.1);

    REQUIRE_THAT(output, WithinAbs(2.0, 1.0e-12));
    REQUIRE(controller.state().saturated);
}

TEST_CASE("conditional integration blocks deeper saturation")
{
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

TEST_CASE("anti-windup none integrates while output is saturated")
{
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

TEST_CASE("anti-windup clamping keeps integral inside configured limits")
{
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

TEST_CASE("back calculation reduces integral pressure while saturated")
{
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

TEST_CASE("runtime gain update changes controller response")
{
    auto config = base_config();
    config.kp = 1.0;
    pidpp::PidController<double> controller{config};

    const double first = controller.update(2.0, 1.0, 0.01);
    controller.set_gains(3.0, 0.0, 0.0);
    const double second = controller.update(2.0, 1.0, 0.01);

    REQUIRE_THAT(first, WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(second, WithinAbs(3.0, 1.0e-12));
}

TEST_CASE("reset clears state")
{
    auto config = base_config();
    config.kp = 1.0;
    config.ki = 1.0;
    pidpp::PidController<double> controller{config};

    static_cast<void>(controller.update(1.0, 0.0, 0.1));
    controller.reset();

    REQUIRE_THAT(controller.state().integral, WithinAbs(0.0, 1.0e-12));
    REQUIRE_FALSE(controller.state().initialized);
}

TEST_CASE("invalid dt returns previous output safely")
{
    auto config = base_config();
    config.kp = 1.0;
    pidpp::PidController<double> controller{config};

    const double first = controller.update(2.0, 1.0, 0.1);
    const double second = controller.update(100.0, 0.0, 0.0);

    REQUIRE_THAT(second, WithinAbs(first, 1.0e-12));
    REQUIRE_FALSE(controller.state().last_dt_valid);
}

TEST_CASE("feedforward can be added without changing PID gains")
{
    auto config = base_config();
    config.feedforward.enabled = true;
    config.feedforward.velocity_gain = 2.0;
    config.feedforward.acceleration_gain = 3.0;
    config.feedforward.static_gain = 1.0;
    pidpp::PidController<double> controller{config};

    const double output = controller.update(0.0, 0.0, 0.01, 4.0, 5.0);

    REQUIRE_THAT(output, WithinAbs(10.0, 1.0e-12));
}

TEST_CASE("update performs no dynamic allocation")
{
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

TEST_CASE("PID state types are lightweight")
{
    STATIC_REQUIRE(std::is_trivially_copyable_v<pidpp::PidState<double>>);
}
