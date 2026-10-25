// Gain scheduling on payload mass (SCARA joint in the horizontal plane).
//
// A SCARA-style joint carries anywhere from 0 to 4 kg at the tip, so its
// inertia varies by about 5x. Gains tuned for the empty arm make the loaded
// arm overshoot and ring. A GainScheduledPidController interpolates gains
// from a table indexed by the payload reported by the gripper, keeping the
// response consistent across loads.
//
// Run with `--csv schedule.csv` to export the runs.

#include <array>
#include <cmath>

#include <pidpp/pidpp.hpp>

#include "common/example_support.hpp"

namespace {

constexpr double scara_link_mass = 1.5;
constexpr double scara_link_length = 0.5;
constexpr double sample_time = 0.001;
constexpr double target_angle = 1.0; // rad

pidpp::PidConfig<double> base_config() {
    return {.output_min = -40.0,
            .output_max = 40.0,
            .integral_min = -0.5,
            .integral_max = 0.5,
            .anti_windup = pidpp::AntiWindupStrategy::Clamping,
            .derivative_mode = pidpp::DerivativeMode::OnMeasurement,
            .derivative_filter_tau = 0.003};
}

/// Gains placing the closed-loop poles near 12 rad/s for a given inertia.
pidpp::PidGains<double> gains_for_inertia(double inertia) {
    constexpr double wn = 12.0;
    return {inertia * wn * wn, inertia * wn * wn * 0.1, 2.0 * 0.9 * inertia * wn};
}

double inertia_with_payload(double payload) {
    return (scara_link_mass * scara_link_length * scara_link_length / 3.0) +
           (payload * scara_link_length * scara_link_length) + 0.02;
}

template <typename Step>
pidpp::StepResponseMetrics<double> run(double payload, Step&& step, example::CsvLog& csv,
                                       double run_id) {
    pidpp::SingleLinkArm<double> joint{scara_link_mass, scara_link_length, 0.2, 0.02};
    joint.set_gravity(0.0); // horizontal plane: no gravity torque
    joint.set_payload_mass(payload);

    std::array<double, 4000> time{};
    std::array<double, 4000> angle{};
    double torque = 0.0;
    for (std::size_t i = 0; i < time.size(); ++i) {
        time[i] = static_cast<double>(i) * sample_time;
        angle[i] = joint.update(torque, sample_time);
        torque = step(joint.angle());
        csv.row({run_id, payload, time[i], angle[i], torque});
    }
    return pidpp::analyze_step_response<double>(time, angle, target_angle);
}

} // namespace

int main(int argc, char** argv) {
    example::CsvLog csv{argc, argv, {"run", "payload", "t", "angle", "torque"}};

    // Schedule: gains for 0, 1, 2, 4 kg payloads.
    const std::array<pidpp::GainSchedulePoint<double>, 4> schedule{{
        {0.0, gains_for_inertia(inertia_with_payload(0.0))},
        {1.0, gains_for_inertia(inertia_with_payload(1.0))},
        {2.0, gains_for_inertia(inertia_with_payload(2.0))},
        {4.0, gains_for_inertia(inertia_with_payload(4.0))},
    }};

    example::title("SCARA joint step with 0 kg and 4 kg payloads");

    bool ok = true;
    double fixed_heavy_overshoot = 0.0;
    double scheduled_heavy_overshoot = 0.0;
    for (const double payload : {0.0, 4.0}) {
        auto fixed_config = base_config();
        fixed_config.set_gains(gains_for_inertia(inertia_with_payload(0.0)));
        pidpp::PidController<double> fixed{fixed_config};
        const auto fixed_metrics = run(
            payload, [&](double q) { return fixed.update(target_angle, q, sample_time); }, csv,
            0.0);

        pidpp::GainScheduledPidController<double, 8> scheduled{base_config()};
        if (!scheduled.set_schedule(schedule)) {
            return 1;
        }
        const auto scheduled_metrics = run(
            payload,
            [&](double q) { return scheduled.update(payload, target_angle, q, sample_time); }, csv,
            1.0);

        std::printf("  payload %.1f kg\n", payload);
        example::print_value("  fixed gains: overshoot", fixed_metrics.overshoot_percent, "%");
        example::print_value("  fixed gains: settling",
                             fixed_metrics.settled ? fixed_metrics.settling_time : -1.0, "s");
        example::print_value("  scheduled: overshoot", scheduled_metrics.overshoot_percent, "%");
        example::print_value("  scheduled: settling",
                             scheduled_metrics.settled ? scheduled_metrics.settling_time : -1.0,
                             "s");

        ok &= scheduled_metrics.settled;
        if (payload > 0.0) {
            fixed_heavy_overshoot = fixed_metrics.overshoot_percent;
            scheduled_heavy_overshoot = scheduled_metrics.overshoot_percent;
        }
    }

    std::printf("  (settling -1 means: not settled within the horizon)\n");
    ok &= example::check(ok, "scheduled controller settles for every payload");
    ok &= example::check(scheduled_heavy_overshoot < 0.5 * fixed_heavy_overshoot,
                         "scheduled gains halve the overshoot with a heavy payload");
    ok &= example::check(scheduled_heavy_overshoot < 10.0, "heavy-payload overshoot below 10 %");
    return example::finish(ok);
}
