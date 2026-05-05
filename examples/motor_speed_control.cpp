#include <iostream>

#include <pidpp/pidpp.hpp>

int main()
{
    // Velocity loop for a simple DC motor model. Derivative-on-measurement
    // avoids setpoint kick when the target speed changes.
    pidpp::PidConfig<double> config{
        .kp = 0.08,
        .ki = 0.6,
        .kd = 0.0005,
        .output_min = -12.0,
        .output_max = 12.0,
        .integral_min = -12.0,
        .integral_max = 12.0,
        .anti_windup = pidpp::AntiWindupStrategy::ConditionalIntegration,
        .derivative_mode = pidpp::DerivativeMode::OnMeasurement,
        .derivative_filter_tau = 0.01
    };

    if (!config.validate()) {
        return 1;
    }

    pidpp::PidController<double> controller{config};
    pidpp::DcMotor<double> motor{2.0, 0.01, 0.1, 0.1, 0.01, 0.001};

    constexpr double dt = 0.001;
    constexpr double target_speed = 50.0;
    double voltage = 0.0;

    for (int i = 0; i < 3000; ++i) {
        const double speed = motor.update(voltage, 0.0, dt);
        voltage = controller.update(target_speed, speed, dt);
    }

    std::cout << "target_speed=" << target_speed
              << " final_speed=" << motor.angular_velocity()
              << " final_voltage=" << voltage << '\n';
    return 0;
}
