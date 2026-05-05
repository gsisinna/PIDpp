#include <iostream>

#include <pidpp/pidpp.hpp>

int main() {
    pidpp::PidConfig<double> config{
        .kp = 1.0, .ki = 0.1, .kd = 0.0, .output_min = -5.0, .output_max = 5.0};

    if (!config.validate()) {
        return 1;
    }

    pidpp::PidController<double> controller{config};
    std::cout << "pidpp " << pidpp::version_string()
              << " command=" << controller.update(1.0, 0.2, 0.01) << '\n';
    return 0;
}
