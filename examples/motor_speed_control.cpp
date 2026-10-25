// Motor speed control (wheel / conveyor / spindle velocity loop).
//
// A PI loop drives a brushed DC motor to a target speed through a 12 V
// supply. A velocity feedforward (kV, the back-EMF/friction voltage per rad/s)
// provides most of the steady-state voltage, so the PI only corrects errors.
// Halfway through, a load torque is applied (a robot wheel hitting a ramp, a
// gripper closing on an object). Conditional integration prevents windup
// while the voltage saturates and the integrator recovers the speed.
//
// Run with `--csv speed.csv` to export the time series.

#include <cmath>

#include <pidpp/pidpp.hpp>

#include "common/example_support.hpp"

int main(int argc, char** argv) {
    pidpp::PidConfig<double> config{.kp = 1.0,
                                    .ki = 4.0,
                                    .kd = 0.0,
                                    .output_min = -12.0,
                                    .output_max = 12.0,
                                    .integral_min = -20.0,
                                    .integral_max = 20.0,
                                    .anti_windup =
                                        pidpp::AntiWindupStrategy::ConditionalIntegration,
                                    .derivative_mode = pidpp::DerivativeMode::OnMeasurement,
                                    // kV = R*b/Kt + Ke = 0.12 V per rad/s
                                    .feedforward = {.velocity_gain = 0.12, .enabled = true}};

    if (const auto result = config.validate(); !result) {
        std::fprintf(stderr, "invalid config: %s\n", result.message());
        return 1;
    }

    pidpp::PidController<double> controller{config};
    // R [ohm], L [H], Kt [Nm/A], Ke [Vs/rad], J [kg m^2], b [Nms/rad]
    pidpp::DcMotor<double> motor{2.0, 0.01, 0.1, 0.1, 0.01, 0.001};
    example::CsvLog csv{argc, argv, {"t", "target", "speed", "voltage", "load"}};

    constexpr double dt = 0.001;
    constexpr double target_speed = 50.0;
    constexpr double load_time = 4.0;
    constexpr double load_torque = 0.05;
    double voltage = 0.0;
    double speed_before_load = 0.0;
    double min_speed_after_load = target_speed;

    for (int i = 0; i < 8000; ++i) {
        const double t = static_cast<double>(i) * dt;
        const double load = t >= load_time ? load_torque : 0.0;
        const double speed = motor.update(voltage, load, dt);
        voltage = controller.update(target_speed, speed, dt, target_speed, 0.0);
        csv.row({t, target_speed, speed, voltage, load});

        if (t < load_time) {
            speed_before_load = speed;
        } else if (speed < min_speed_after_load) {
            min_speed_after_load = speed;
        }
    }

    example::title("DC motor speed loop with load disturbance");
    example::print_value("target speed", target_speed, "rad/s");
    example::print_value("speed before load", speed_before_load, "rad/s");
    example::print_value("min speed after load step", min_speed_after_load, "rad/s");
    example::print_value("final speed", motor.angular_velocity(), "rad/s");
    example::print_value("final voltage", voltage, "V");

    bool ok = example::check(std::abs(speed_before_load - target_speed) < 0.5,
                             "reaches target speed before the disturbance");
    ok &= example::check(std::abs(motor.angular_velocity() - target_speed) < 0.5,
                         "integral action rejects the load torque");
    return example::finish(ok);
}
