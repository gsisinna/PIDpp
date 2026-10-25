// Relay autotuning of a motor speed loop.
//
// Instead of hand-tuning, run an Astrom-Hagglund relay experiment: the relay
// toggles the motor voltage around an operating point (6 V +/- 2 V) and the
// speed settles into a limit cycle whose period and amplitude give the
// ultimate gain Ku and period Tu. Classic rules turn (Ku, Tu) into PI gains,
// which are then validated with a closed-loop step.
//
// Speed is estimated from an encoder through a 5 ms low-pass filter, as on
// most embedded drives; that filter adds the phase lag that makes the limit
// cycle well defined.

#include <array>
#include <cmath>
#include <vector>

#include <pidpp/pidpp.hpp>

#include "common/example_support.hpp"

namespace {

constexpr double dt = 0.0002; // 5 kHz
constexpr double operating_voltage = 6.0;

struct SpeedSensor {
    pidpp::LowPassFilter<double> filter{0.005};

    double read(double true_speed) {
        return filter.update(true_speed, dt);
    }
};

} // namespace

int main(int /*argc*/, char** /*argv*/) {
    // --- 1. Relay experiment around the operating point --------------------
    pidpp::DcMotor<double> motor{2.0, 0.01, 0.1, 0.1, 0.01, 0.001};
    SpeedSensor sensor;
    const double operating_speed = motor.steady_state_speed(operating_voltage);

    // Spin up to the operating point first, as a real tuner would.
    for (int i = 0; i < 50000; ++i) {
        motor.update(operating_voltage, 0.0, dt);
        static_cast<void>(sensor.read(motor.angular_velocity()));
    }

    pidpp::RelayAutotuner<double> tuner{{.relay_amplitude = 2.0,
                                         .hysteresis = 0.2,
                                         .output_bias = operating_voltage,
                                         .cycles = 4,
                                         .timeout = 20.0}};
    if (!tuner.config().validate()) {
        return 1;
    }

    double voltage = operating_voltage;
    for (int i = 0; i < 200000 && !tuner.complete() && !tuner.result().timed_out; ++i) {
        motor.update(voltage, 0.0, dt);
        voltage = tuner.update(operating_speed, sensor.read(motor.angular_velocity()), dt);
    }

    example::title("Relay experiment");
    const auto& result = tuner.result();
    example::print_value("ultimate gain Ku", result.ultimate_gain, "V/(rad/s)");
    example::print_value("ultimate period Tu", result.ultimate_period, "s");
    example::print_value("oscillation amplitude", result.oscillation_amplitude, "rad/s");
    bool ok = example::check(tuner.complete(), "limit cycle identified");
    if (!ok) {
        return example::finish(false);
    }

    // --- 2. Gains from several tuning rules ---------------------------------
    example::title("Suggested gains");
    struct Rule {
        const char* name;
        pidpp::TuningRule rule;
    };
    constexpr std::array<Rule, 4> rules{{
        {"Ziegler-Nichols PI", pidpp::TuningRule::ZieglerNicholsPi},
        {"Ziegler-Nichols PID", pidpp::TuningRule::ZieglerNicholsPid},
        {"Tyreus-Luyben PI", pidpp::TuningRule::TyreusLuybenPi},
        {"No-overshoot PID", pidpp::TuningRule::NoOvershootPid},
    }};
    for (const auto& rule : rules) {
        const auto gains = tuner.gains(rule.rule);
        std::printf("  %-22s kp=%8.4f ki=%9.4f kd=%8.5f\n", rule.name, gains.kp, gains.ki,
                    gains.kd);
    }

    // --- 3. Closed-loop validation with Tyreus-Luyben PI --------------------
    pidpp::PidConfig<double> config{.output_min = -12.0,
                                    .output_max = 12.0,
                                    .integral_min = -100.0,
                                    .integral_max = 100.0,
                                    .anti_windup = pidpp::AntiWindupStrategy::Clamping};
    config.set_gains(tuner.gains(pidpp::TuningRule::TyreusLuybenPi));
    ok &= example::check(static_cast<bool>(config.validate()), "tuned config is valid");

    pidpp::PidController<double> controller{config};
    motor.reset();
    sensor = {};
    constexpr double target = 40.0;
    std::vector<double> time;
    std::vector<double> speed;
    voltage = 0.0;
    for (int i = 0; i < 25000; ++i) {
        motor.update(voltage, 0.0, dt);
        const double measured = sensor.read(motor.angular_velocity());
        voltage = controller.update(target, measured, dt);
        time.push_back(static_cast<double>(i) * dt);
        speed.push_back(motor.angular_velocity());
    }
    const auto metrics = pidpp::analyze_step_response<double>(time, speed, target);

    example::title("Closed-loop step with Tyreus-Luyben PI (0 -> 40 rad/s)");
    example::print_value("overshoot", metrics.overshoot_percent, "%");
    example::print_value("settling time (2%)", metrics.settling_time, "s");
    example::print_value("final speed", speed.back(), "rad/s");

    ok &= example::check(metrics.settled, "tuned loop settles");
    ok &= example::check(std::abs(speed.back() - target) < 0.2, "tracks the target speed");
    return example::finish(ok);
}
