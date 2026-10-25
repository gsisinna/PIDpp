#include <benchmark/benchmark.h>

#include <array>

#include <pidpp/pidpp.hpp>

namespace {

static void update_latency(benchmark::State& state) {
    pidpp::PidConfig<double> config{.kp = 1.2,
                                    .ki = 0.4,
                                    .kd = 0.05,
                                    .output_min = -12.0,
                                    .output_max = 12.0,
                                    .integral_min = -10.0,
                                    .integral_max = 10.0,
                                    .anti_windup =
                                        pidpp::AntiWindupStrategy::ConditionalIntegration,
                                    .derivative_mode = pidpp::DerivativeMode::OnMeasurement,
                                    .derivative_filter_tau = 0.02};
    pidpp::PidController<double> controller{config};

    for (auto _ : state) {
        benchmark::DoNotOptimize(controller.update(10.0, 8.5, 0.001));
    }
}

BENCHMARK(update_latency);

static void pid_update_without_filter(benchmark::State& state) {
    pidpp::PidConfig<double> config{.kp = 1.2,
                                    .ki = 0.4,
                                    .kd = 0.05,
                                    .output_min = -12.0,
                                    .output_max = 12.0,
                                    .integral_min = -10.0,
                                    .integral_max = 10.0,
                                    .anti_windup = pidpp::AntiWindupStrategy::None,
                                    .derivative_mode = pidpp::DerivativeMode::OnError};
    pidpp::PidController<double> controller{config};

    for (auto _ : state) {
        benchmark::DoNotOptimize(controller.update(10.0, 8.5, 0.001));
    }
}

static void pid_update_with_derivative_filter(benchmark::State& state) {
    pidpp::PidConfig<double> config{.kp = 1.2,
                                    .ki = 0.4,
                                    .kd = 0.05,
                                    .output_min = -12.0,
                                    .output_max = 12.0,
                                    .integral_min = -10.0,
                                    .integral_max = 10.0,
                                    .anti_windup =
                                        pidpp::AntiWindupStrategy::ConditionalIntegration,
                                    .derivative_mode = pidpp::DerivativeMode::OnMeasurement,
                                    .derivative_filter_tau = 0.02};
    pidpp::PidController<double> controller{config};

    for (auto _ : state) {
        benchmark::DoNotOptimize(controller.update(10.0, 8.5, 0.001));
    }
}

static void pi_update(benchmark::State& state) {
    pidpp::PidConfig<double> config{.kp = 1.0,
                                    .ki = 0.25,
                                    .kd = 0.0,
                                    .output_min = -12.0,
                                    .output_max = 12.0,
                                    .integral_min = -10.0,
                                    .integral_max = 10.0,
                                    .anti_windup = pidpp::AntiWindupStrategy::Clamping};
    pidpp::PidController<double> controller{config};

    for (auto _ : state) {
        benchmark::DoNotOptimize(controller.update(10.0, 8.5, 0.001));
    }
}

static void saturated_pid_update(benchmark::State& state) {
    pidpp::PidConfig<double> config{.kp = 100.0,
                                    .ki = 10.0,
                                    .kd = 1.0,
                                    .output_min = -1.0,
                                    .output_max = 1.0,
                                    .integral_min = -2.0,
                                    .integral_max = 2.0,
                                    .anti_windup = pidpp::AntiWindupStrategy::BackCalculation};
    pidpp::PidController<double> controller{config};

    for (auto _ : state) {
        benchmark::DoNotOptimize(controller.update(10.0, 0.0, 0.001));
    }
}

static void cascaded_update(benchmark::State& state) {
    pidpp::CascadedPidConfig<double> config{};
    config.outer = {.kp = 15.0, .output_min = -60.0, .output_max = 60.0};
    config.inner = {.kp = 4.0,
                    .ki = 60.0,
                    .output_min = -24.0,
                    .output_max = 24.0,
                    .integral_min = -5.0,
                    .integral_max = 5.0,
                    .anti_windup = pidpp::AntiWindupStrategy::Clamping};
    pidpp::CascadedPidController<double> controller{config};

    for (auto _ : state) {
        benchmark::DoNotOptimize(controller.update(1.0, 0.5, 2.0, 0.0005));
    }
}

static void gain_scheduled_update(benchmark::State& state) {
    pidpp::GainScheduledPidController<double, 8> controller{
        {.output_min = -12.0, .output_max = 12.0, .integral_min = -10.0, .integral_max = 10.0}};
    const std::array<pidpp::GainSchedulePoint<double>, 4> points{{
        {0.0, {1.0, 0.1, 0.01}},
        {1.0, {2.0, 0.2, 0.02}},
        {2.0, {3.0, 0.3, 0.03}},
        {4.0, {5.0, 0.5, 0.05}},
    }};
    static_cast<void>(controller.set_schedule(points));

    for (auto _ : state) {
        benchmark::DoNotOptimize(controller.update(2.5, 10.0, 8.5, 0.001));
    }
}

static void trapezoidal_profile_sample(benchmark::State& state) {
    pidpp::TrapezoidalProfile<double> profile;
    static_cast<void>(profile.plan(0.0, 10.0, {.max_velocity = 2.0, .max_acceleration = 1.0}));
    double t = 0.0;

    for (auto _ : state) {
        benchmark::DoNotOptimize(profile.sample(t));
        t = t > 7.0 ? 0.0 : t + 0.001;
    }
}

BENCHMARK(pid_update_without_filter);
BENCHMARK(pid_update_with_derivative_filter);
BENCHMARK(pi_update);
BENCHMARK(saturated_pid_update);
BENCHMARK(cascaded_update);
BENCHMARK(gain_scheduled_update);
BENCHMARK(trapezoidal_profile_sample);

} // namespace
