#include <array>
#include <iostream>

#include <pidpp/pidpp.hpp>

int main() {
    std::array<double, 1000> time{};
    std::array<double, 1000> output{};
    pidpp::FirstOrderPlant<double> plant{1.0, 0.2};

    // Metrics are offline helpers. They are intentionally kept out of the
    // real-time controller update path.
    constexpr double dt = 0.001;
    for (std::size_t i = 0; i < output.size(); ++i) {
        time[i] = static_cast<double>(i) * dt;
        output[i] = plant.update(1.0, dt);
    }

    const auto metrics = pidpp::analyze_step_response<double>(time, output, 1.0);

    std::cout << "final=" << metrics.final_value
              << " overshoot_percent=" << metrics.overshoot_percent
              << " settling_time=" << metrics.settling_time << '\n';
    return 0;
}
