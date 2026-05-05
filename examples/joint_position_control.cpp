#include <iostream>

#include <pidpp/pidpp.hpp>

int main() {
    // Position loop around a second-order joint model. Back-calculation limits
    // integral buildup when the torque command saturates.
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
    pidpp::MassSpringDamper<double> joint{1.2, 1.4, 0.0};

    constexpr double dt = 0.001;
    constexpr double target_position = 1.57;
    double torque = 0.0;

    for (int i = 0; i < 5000; ++i) {
        const double position = joint.update(torque, dt);
        torque = controller.update(target_position, position, dt);
    }

    std::cout << "target_position=" << target_position << " final_position=" << joint.position()
              << " final_velocity=" << joint.velocity() << '\n';
    return 0;
}
