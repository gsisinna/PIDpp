// Joint position control (horizontal revolute joint, no gravity).
//
// A PID loop commands torque to rotate a joint by 90 degrees. Derivative on
// measurement avoids the derivative kick of the setpoint step and
// back-calculation limits integral buildup while the torque saturates.
//
// Run with `--csv joint.csv` to export the time series.

#include <cmath>
#include <numbers>
#include <vector>

#include <pidpp/pidpp.hpp>

#include "common/example_support.hpp"

int main(int argc, char** argv) {
    pidpp::PidConfig<double> config{.kp = 45.0,
                                    .ki = 2.0,
                                    .kd = 8.0,
                                    .output_min = -60.0,
                                    .output_max = 60.0,
                                    .integral_min = -5.0,
                                    .integral_max = 5.0,
                                    .anti_windup = pidpp::AntiWindupStrategy::BackCalculation,
                                    .derivative_mode = pidpp::DerivativeMode::OnMeasurement,
                                    .derivative_filter_tau = 0.02};

    if (!config.validate()) {
        return 1;
    }

    pidpp::PidController<double> controller{config};
    // Rotor + link inertia 1.2 kg m^2, viscous friction 1.4 Nms/rad, no spring.
    pidpp::MassSpringDamper<double> joint{1.2, 1.4, 0.0};
    example::CsvLog csv{argc, argv, {"t", "target", "position", "torque"}};

    constexpr double dt = 0.001;
    constexpr double target_position = std::numbers::pi / 2.0;
    double torque = 0.0;
    std::vector<double> time;
    std::vector<double> position;

    for (int i = 0; i < 5000; ++i) {
        const double t = static_cast<double>(i) * dt;
        const double angle = joint.update(torque, dt);
        torque = controller.update(target_position, angle, dt);
        time.push_back(t);
        position.push_back(angle);
        csv.row({t, target_position, angle, torque});
    }

    const auto metrics = pidpp::analyze_step_response<double>(time, position, target_position);

    example::title("Joint position step (90 deg)");
    example::print_value("target", target_position, "rad");
    example::print_value("final position", joint.position(), "rad");
    example::print_value("overshoot", metrics.overshoot_percent, "%");
    example::print_value("rise time (10-90%)", metrics.rise_time, "s");
    example::print_value("settling time (2%)", metrics.settling_time, "s");

    bool ok = example::check(std::abs(joint.position() - target_position) < 5.0e-3,
                             "joint reaches the target");
    ok &= example::check(metrics.settled && metrics.settling_time < 3.0, "settles within 3 s");
    ok &= example::check(metrics.overshoot_percent < 10.0, "overshoot below 10 %");
    return example::finish(ok);
}
