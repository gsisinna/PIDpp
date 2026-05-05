#include <iostream>

#include <pidpp/pidpp.hpp>

int main()
{
    pidpp::PidConfig<double> config{
        .kp = 1.2,
        .ki = 0.4,
        .kd = 0.05,
        .output_min = -12.0,
        .output_max = 12.0,
        .anti_windup = pidpp::AntiWindupStrategy::ConditionalIntegration,
        .derivative_mode = pidpp::DerivativeMode::OnMeasurement,
        .derivative_filter_tau = 0.02
    };

    const auto validation = config.validate();
    if (!validation) {
        std::cerr << "Invalid PID config: " << validation.message() << '\n';
        return 1;
    }

    pidpp::PidController<double> controller{config};

    // One control step: command drives measurement toward the setpoint.
    const double command = controller.update(
        10.0,
        8.5,
        0.001);

    std::cout << "command=" << command
              << " error=" << controller.state().error << '\n';
    return 0;
}
