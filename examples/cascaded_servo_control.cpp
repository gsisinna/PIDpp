// Cascaded position/velocity servo (industrial servo drive, robot joint).
//
// The classic servo structure: an outer P position loop produces a velocity
// setpoint, an inner PI velocity loop produces the motor voltage. A
// trapezoidal profile moves the motor one full revolution; its velocity and
// acceleration feed forward into both loops so the PID terms only correct
// model errors. The outer loop runs at half the inner rate, as it often does
// on real drives.
//
// Run with `--csv servo.csv` to export the time series.

#include <algorithm>
#include <cmath>
#include <numbers>

#include <pidpp/pidpp.hpp>

#include "common/example_support.hpp"

int main(int argc, char** argv) {
    // Motor: R = 2 ohm, L = 10 mH, Kt = Ke = 0.1, J = 0.01 kg m^2, b = 0.001.
    pidpp::DcMotor<double> motor{2.0, 0.01, 0.1, 0.1, 0.01, 0.001};

    pidpp::CascadedPidConfig<double> config{};
    // Position loop: P plus profile velocity feedforward (output = rad/s).
    config.outer = {.kp = 15.0,
                    .output_min = -60.0,
                    .output_max = 60.0,
                    .feedforward = {.velocity_gain = 1.0, .enabled = true}};
    // Velocity loop: PI plus model feedforward (output = volts).
    //   V = (R*b/Kt + Ke) * w + (R*J/Kt) * a
    config.inner = {
        .kp = 4.0,
        .ki = 60.0,
        .output_min = -24.0,
        .output_max = 24.0,
        .integral_min = -5.0,
        .integral_max = 5.0,
        .anti_windup = pidpp::AntiWindupStrategy::Clamping,
        .feedforward = {.velocity_gain = 0.12, .acceleration_gain = 0.2, .enabled = true}};
    config.outer_decimation = 2;

    if (const auto result = config.validate(); !result) {
        std::fprintf(stderr, "invalid config: %s\n", result.message());
        return 1;
    }

    pidpp::CascadedPidController<double> servo{config};
    pidpp::TrapezoidalProfile<double> profile;
    const double goal = 2.0 * std::numbers::pi;
    if (!profile.plan(0.0, goal, {.max_velocity = 30.0, .max_acceleration = 80.0})) {
        return 1;
    }

    example::CsvLog csv{
        argc, argv, {"t", "ref_pos", "pos", "ref_vel", "vel", "vel_setpoint", "voltage"}};

    constexpr double dt = 0.0005; // 2 kHz inner loop, 1 kHz outer loop
    const auto steps = static_cast<int>((profile.duration() + 0.5) / dt);
    double voltage = 0.0;
    double max_tracking_error = 0.0;

    for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i) * dt;
        motor.update(voltage, 0.0, dt);
        const auto ref = profile.sample(t);
        voltage = servo.update(ref.position, motor.position(), motor.angular_velocity(), dt,
                               ref.velocity, ref.acceleration);
        max_tracking_error =
            std::max(max_tracking_error, std::abs(ref.position - motor.position()));
        csv.row({t, ref.position, motor.position(), ref.velocity, motor.angular_velocity(),
                 servo.inner_setpoint(), voltage});
    }

    const double final_error = std::abs(goal - motor.position());

    example::title("Cascaded servo: one revolution with a trapezoidal profile");
    example::print_value("move duration", profile.duration(), "s");
    example::print_value("max tracking error", max_tracking_error, "rad");
    example::print_value("final position error", final_error, "rad");
    example::print_value("final velocity", motor.angular_velocity(), "rad/s");

    bool ok = example::check(max_tracking_error < 0.05, "tracking error below 0.05 rad");
    ok &= example::check(final_error < 1.0e-3, "final position error below 1 mrad");
    return example::finish(ok);
}
