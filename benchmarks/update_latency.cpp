#include <benchmark/benchmark.h>

#include <new>

#include <pidpp/pidpp.hpp>

namespace {

static void update_latency(benchmark::State& state)
{
    pidpp::PidConfig<double> config{
        .kp = 1.2,
        .ki = 0.4,
        .kd = 0.05,
        .output_min = -12.0,
        .output_max = 12.0,
        .integral_min = -10.0,
        .integral_max = 10.0,
        .anti_windup = pidpp::AntiWindupStrategy::ConditionalIntegration,
        .derivative_mode = pidpp::DerivativeMode::OnMeasurement,
        .derivative_filter_tau = 0.02
    };
    pidpp::PidController<double> controller{config};

    for (auto _ : state) {
        benchmark::DoNotOptimize(controller.update(10.0, 8.5, 0.001));
    }
}

BENCHMARK(update_latency);

static void pid_update_without_filter(benchmark::State& state)
{
    pidpp::PidConfig<double> config{
        .kp = 1.2,
        .ki = 0.4,
        .kd = 0.05,
        .output_min = -12.0,
        .output_max = 12.0,
        .integral_min = -10.0,
        .integral_max = 10.0,
        .anti_windup = pidpp::AntiWindupStrategy::None,
        .derivative_mode = pidpp::DerivativeMode::OnError
    };
    pidpp::PidController<double> controller{config};

    for (auto _ : state) {
        benchmark::DoNotOptimize(controller.update(10.0, 8.5, 0.001));
    }
}

static void pid_update_with_derivative_filter(benchmark::State& state)
{
    pidpp::PidConfig<double> config{
        .kp = 1.2,
        .ki = 0.4,
        .kd = 0.05,
        .output_min = -12.0,
        .output_max = 12.0,
        .integral_min = -10.0,
        .integral_max = 10.0,
        .anti_windup = pidpp::AntiWindupStrategy::ConditionalIntegration,
        .derivative_mode = pidpp::DerivativeMode::OnMeasurement,
        .derivative_filter_tau = 0.02
    };
    pidpp::PidController<double> controller{config};

    for (auto _ : state) {
        benchmark::DoNotOptimize(controller.update(10.0, 8.5, 0.001));
    }
}

static void pi_update(benchmark::State& state)
{
    pidpp::PidConfig<double> config{
        .kp = 1.0,
        .ki = 0.25,
        .kd = 0.0,
        .output_min = -12.0,
        .output_max = 12.0,
        .integral_min = -10.0,
        .integral_max = 10.0,
        .anti_windup = pidpp::AntiWindupStrategy::Clamping
    };
    pidpp::PidController<double> controller{config};

    for (auto _ : state) {
        benchmark::DoNotOptimize(controller.update(10.0, 8.5, 0.001));
    }
}

static void saturated_pid_update(benchmark::State& state)
{
    pidpp::PidConfig<double> config{
        .kp = 100.0,
        .ki = 10.0,
        .kd = 1.0,
        .output_min = -1.0,
        .output_max = 1.0,
        .integral_min = -2.0,
        .integral_max = 2.0,
        .anti_windup = pidpp::AntiWindupStrategy::BackCalculation
    };
    pidpp::PidController<double> controller{config};

    for (auto _ : state) {
        benchmark::DoNotOptimize(controller.update(10.0, 0.0, 0.001));
    }
}

BENCHMARK(pid_update_without_filter);
BENCHMARK(pid_update_with_derivative_filter);
BENCHMARK(pi_update);
BENCHMARK(saturated_pid_update);

} // namespace
