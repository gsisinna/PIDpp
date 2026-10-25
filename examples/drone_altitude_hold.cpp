// Multirotor altitude hold.
//
// The vertical loop of a 1.2 kg quadrotor: take off to 10 m, hold, then drop
// a 0.3 kg package at t = 15 s. Hover thrust (m*g) is supplied as static
// feedforward so the PID only handles deviations. The altitude setpoint is
// slew-rate limited for a smooth climb, the thrust command is bounded to the
// motors' range, and its rate is limited to protect the ESCs.
//
// Run with `--csv altitude.csv` to export the time series.

#include <algorithm>
#include <cmath>

#include <pidpp/pidpp.hpp>

#include "common/example_support.hpp"

int main(int argc, char** argv) {
    constexpr double mass = 1.2;             // kg, including the package
    constexpr double package_mass = 0.3;     // kg
    constexpr double max_thrust = 25.0;      // N, four motors at full throttle
    constexpr double target_altitude = 10.0; // m
    constexpr double drop_time = 15.0;       // s
    constexpr double dt = 0.002;             // 500 Hz

    pidpp::VerticalThrustPlant<double> drone{mass, 0.4, 0.04};

    pidpp::PidConfig<double> config{
        .kp = 10.0,
        .ki = 2.0,
        .kd = 3.0,
        .output_min = 0.0, // propellers cannot pull down
        .output_max = max_thrust,
        .integral_min = -5.0,
        .integral_max = 5.0,
        .anti_windup = pidpp::AntiWindupStrategy::ConditionalIntegration,
        .derivative_mode = pidpp::DerivativeMode::OnMeasurement,
        .derivative_filter_tau = 0.02,
        .feedforward = {.static_gain = drone.hover_thrust(), .enabled = true},
        .output_rate_limit = 80.0}; // N/s
    if (const auto result = config.validate(); !result) {
        std::fprintf(stderr, "invalid config: %s\n", result.message());
        return 1;
    }

    pidpp::PidController<double> controller{config};
    pidpp::SlewRateLimiter<double> climb{1.0, 1.0, 0.0}; // m/s up, m/s down
    example::CsvLog csv{argc, argv, {"t", "setpoint", "altitude", "thrust_cmd"}};

    double thrust = 0.0;
    double peak_altitude = 0.0;
    double max_drop_deviation = 0.0;
    bool package_dropped = false;

    for (int i = 0; i < 20000; ++i) { // 40 s
        const double t = static_cast<double>(i) * dt;
        if (!package_dropped && t >= drop_time) {
            drone.set_mass(mass - package_mass);
            package_dropped = true;
        }
        drone.update(thrust, dt);
        const double setpoint = climb.update(target_altitude, dt);
        thrust = controller.update(setpoint, drone.altitude(), dt);

        if (!package_dropped) {
            peak_altitude = std::max(peak_altitude, drone.altitude());
        } else {
            max_drop_deviation =
                std::max(max_drop_deviation, std::abs(drone.altitude() - target_altitude));
        }
        csv.row({t, setpoint, drone.altitude(), thrust});
    }

    const double overshoot = (peak_altitude - target_altitude) / target_altitude * 100.0;
    const double final_error = std::abs(drone.altitude() - target_altitude);

    example::title("Quadrotor altitude hold with package drop");
    example::print_value("peak altitude", peak_altitude, "m");
    example::print_value("climb overshoot", overshoot, "%");
    example::print_value("max deviation after drop", max_drop_deviation, "m");
    example::print_value("final altitude", drone.altitude(), "m");
    example::print_value("final thrust", thrust, "N");
    example::print_value("hover thrust (new mass)", drone.hover_thrust(), "N");

    bool ok = example::check(overshoot < 5.0, "climb overshoot below 5 %");
    ok &= example::check(max_drop_deviation < 0.5, "package drop disturbance below 0.5 m");
    ok &= example::check(final_error < 0.02, "integrator re-trims hover thrust (< 2 cm)");
    return example::finish(ok);
}
