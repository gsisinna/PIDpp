// Robot arm joint with gravity compensation.
//
// A shoulder joint lifts a 0.6 m link from hanging straight down (-90 deg) to
// 30 deg above horizontal and holds it; at t = 4 s the gripper picks up a
// 0.5 kg object. Gravity torque changes with the joint angle during the move,
// so a plain PID lags behind until its integrator catches up, and the
// integrator then overshoots once the arm stops. A model-based gravity
// feedforward, m*g*l*cos(q), supplies that torque directly; the integrator is
// left with the model error (5 % here) and the unknown payload.
//
// Run with `--csv arm.csv` to export both runs.

#include <cmath>
#include <numbers>
#include <vector>

#include <pidpp/pidpp.hpp>

#include "common/example_support.hpp"

namespace {

constexpr double arm_link_mass = 2.0;      // kg
constexpr double arm_link_length = 0.6;    // m
constexpr double joint_damping = 0.8;      // N m s / rad
constexpr double arm_rotor_inertia = 0.05; // kg m^2 reflected through the gearbox
constexpr double sample_time = 0.001;
constexpr double pickup_time = 4.0;

struct RunResult {
    double move_rms_error; // during the move and the following 0.5 s
    double move_max_error;
    double final_error;
};

RunResult run(bool gravity_feedforward, example::CsvLog& csv) {
    // Derivative on error: the reference is a smooth profile (no setpoint
    // kick), and damping relative to the moving reference avoids the lag that
    // derivative-on-measurement adds while tracking.
    pidpp::PidConfig<double> config{.kp = 60.0,
                                    .ki = 80.0,
                                    .kd = 8.0,
                                    .output_min = -40.0,
                                    .output_max = 40.0,
                                    .integral_min = -1.0,
                                    .integral_max = 1.0,
                                    .anti_windup = pidpp::AntiWindupStrategy::Clamping,
                                    .derivative_mode = pidpp::DerivativeMode::OnError,
                                    .derivative_filter_tau = 0.005};
    pidpp::PidController<double> controller{config};

    pidpp::SingleLinkArm<double> arm{arm_link_mass, arm_link_length, joint_damping,
                                     arm_rotor_inertia};
    arm.reset(-std::numbers::pi / 2.0);
    // The controller's model knows the link but not the payload, and is 5 % off.
    const pidpp::SingleLinkArm<double> model{arm_link_mass * 1.05, arm_link_length, 0.0};

    pidpp::TrapezoidalProfile<double> profile;
    static_cast<void>(profile.plan(-std::numbers::pi / 2.0, pidpp::deg_to_rad(30.0),
                                   {.max_velocity = 1.5, .max_acceleration = 4.0}));
    const double move_window = profile.duration() + 0.5;

    std::vector<double> reference;
    std::vector<double> measured;
    double torque = 0.0;
    double max_move_error = 0.0;

    for (int i = 0; i < 8000; ++i) {
        const double t = static_cast<double>(i) * sample_time;
        if (t >= pickup_time) {
            arm.set_payload_mass(0.5);
        }
        arm.update(torque, sample_time);
        const double target = profile.sample(t).position;
        const double ff = gravity_feedforward ? model.gravity_torque(arm.angle()) : 0.0;
        torque = controller.update_with_feedforward(target, arm.angle(), sample_time, ff);

        if (t <= move_window) {
            reference.push_back(target);
            measured.push_back(arm.angle());
            max_move_error = std::max(max_move_error, std::abs(target - arm.angle()));
        }
        csv.row({gravity_feedforward ? 1.0 : 0.0, t, target, arm.angle(), torque});
    }

    const auto tracking = pidpp::analyze_tracking<double>(reference, measured);
    return {tracking.rms_error, max_move_error, std::abs(profile.goal() - arm.angle())};
}

} // namespace

int main(int argc, char** argv) {
    example::CsvLog csv{argc, argv, {"gravity_ff", "t", "reference", "angle", "torque"}};

    const RunResult pid_only = run(false, csv);
    const RunResult compensated = run(true, csv);

    example::title("Arm joint: PID vs PID + gravity feedforward (payload pickup at 4 s)");
    example::print_value("PID only: move RMS error", pid_only.move_rms_error, "rad");
    example::print_value("PID only: move max error", pid_only.move_max_error, "rad");
    example::print_value("PID only: final error", pid_only.final_error, "rad");
    example::print_value("PID + FF: move RMS error", compensated.move_rms_error, "rad");
    example::print_value("PID + FF: move max error", compensated.move_max_error, "rad");
    example::print_value("PID + FF: final error", compensated.final_error, "rad");

    bool ok = example::check(compensated.move_rms_error * 2.0 < pid_only.move_rms_error,
                             "gravity feedforward halves the tracking error during the move");
    ok &= example::check(compensated.final_error < 2.0e-3 && pid_only.final_error < 2.0e-3,
                         "integrator removes the unmodeled payload error in both runs");
    return example::finish(ok);
}
