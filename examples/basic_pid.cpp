// Minimal PID usage: configure, validate, construct, update.

#include <iostream>

#include <pidpp/pidpp.hpp>

int main() {
    pidpp::PidConfig<double> config{.kp = 1.2,
                                    .ki = 0.4,
                                    .kd = 0.05,
                                    .output_min = -12.0,
                                    .output_max = 12.0,
                                    .anti_windup =
                                        pidpp::AntiWindupStrategy::ConditionalIntegration,
                                    .derivative_mode = pidpp::DerivativeMode::OnMeasurement,
                                    .derivative_filter_tau = 0.02};

    // Validate once, outside the control loop. Validation never throws.
    const auto validation = config.validate();
    if (!validation) {
        std::cerr << "Invalid PID config: " << validation.message() << '\n';
        return 1;
    }

    pidpp::PidController<double> controller{config};

    // One control step: command drives measurement toward the setpoint.
    const double command = controller.update(10.0, 8.5, 0.001);
    const auto& state = controller.state();

    std::cout << "pidpp " << pidpp::version_string() << '\n'
              << "command=" << command << " error=" << state.error
              << " P=" << state.proportional_term << " I=" << state.integral_term
              << " D=" << state.derivative_term << '\n';
    return command > 0.0 ? 0 : 1;
}
