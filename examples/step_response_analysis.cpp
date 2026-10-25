// Offline step-response analysis: compare P, PI, and PID on a lag plant.
//
// A second-order process (two first-order lags, e.g. a heater with a sensor
// lag, or a velocity loop with a filtered encoder) is controlled by three
// controllers. The metrics helpers summarize each response the way a tuning
// report would: overshoot, rise and settling time, steady-state error, IAE.
// Metrics are offline helpers, intentionally kept out of the real-time path.

#include <array>
#include <cmath>

#include <pidpp/pidpp.hpp>

#include "common/example_support.hpp"

namespace {

constexpr double sample_time = 0.001;
constexpr std::size_t samples = 6000;
constexpr double target_value = 1.0;

pidpp::StepResponseMetrics<double> simulate(const pidpp::PidConfig<double>& config) {
    pidpp::PidController<double> controller{config};
    pidpp::FirstOrderPlant<double> actuator{1.0, 0.3};
    pidpp::FirstOrderPlant<double> process{1.0, 0.1};

    std::array<double, samples> time{};
    std::array<double, samples> output{};
    double command = 0.0;
    for (std::size_t i = 0; i < samples; ++i) {
        time[i] = static_cast<double>(i) * sample_time;
        output[i] = process.update(actuator.update(command, sample_time), sample_time);
        command = controller.update(target_value, output[i], sample_time);
    }
    return pidpp::analyze_step_response<double>(time, output, target_value);
}

void print(const char* name, const pidpp::StepResponseMetrics<double>& m) {
    std::printf("  %-4s overshoot=%6.2f %%  rise=%6.3f s  settling=%s%6.3f s  sse=%8.5f  "
                "IAE=%6.4f\n",
                name, m.overshoot_percent, m.rise_time, m.settled ? "" : ">", m.settling_time,
                m.steady_state_error, m.integral_absolute_error);
}

} // namespace

int main() {
    const pidpp::PidConfig<double> base{.output_min = -10.0,
                                        .output_max = 10.0,
                                        .integral_min = -10.0,
                                        .integral_max = 10.0,
                                        .anti_windup = pidpp::AntiWindupStrategy::Clamping,
                                        .derivative_mode = pidpp::DerivativeMode::OnMeasurement,
                                        .derivative_filter_tau = 0.01};

    auto p = base;
    p.kp = 4.0;
    auto pi = base;
    pi.set_gains({3.0, 6.0, 0.0});
    auto pid = base;
    pid.set_gains({10.0, 15.0, 0.5});

    const auto p_metrics = simulate(p);
    const auto pi_metrics = simulate(pi);
    const auto pid_metrics = simulate(pid);

    example::title("Step response comparison on a two-lag process");
    print("P", p_metrics);
    print("PI", pi_metrics);
    print("PID", pid_metrics);

    bool ok = example::check(std::abs(p_metrics.steady_state_error - (target_value / 5.0)) < 0.01,
                             "P-only leaves the expected steady-state error 1/(1+Kp)");
    ok &= example::check(std::abs(pi_metrics.steady_state_error) < 1.0e-3,
                         "integral action removes steady-state error");
    ok &=
        example::check(pid_metrics.settled && pid_metrics.settling_time < pi_metrics.settling_time,
                       "derivative action speeds up settling");
    return example::finish(ok);
}
