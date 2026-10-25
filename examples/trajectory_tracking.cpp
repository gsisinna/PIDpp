// Trajectory tracking with feedforward (pick-and-place joint move).
//
// The same PID gains follow a trapezoidal joint trajectory twice: once with
// feedback only, once with velocity/acceleration feedforward derived from the
// joint model (kV = viscous friction, kA = inertia). Feedback alone always
// lags a moving reference; feedforward supplies the expected torque up front
// and leaves the PID to reject the residual error.
//
// Run with `--csv tracking.csv` to export both runs.

#include <cmath>
#include <numbers>
#include <vector>

#include <pidpp/pidpp.hpp>

#include "common/example_support.hpp"

namespace {

constexpr double inertia = 1.2;  // kg m^2
constexpr double friction = 1.4; // N m s / rad
constexpr double dt = 0.001;

struct RunResult {
    pidpp::TrackingMetrics<double> metrics;
    double final_error;
};

RunResult run(const pidpp::PidConfig<double>& config,
              const pidpp::TrapezoidalProfile<double>& profile, example::CsvLog& csv,
              double run_id) {
    pidpp::PidController<double> controller{config};
    pidpp::MassSpringDamper<double> joint{inertia, friction, 0.0};
    std::vector<double> reference;
    std::vector<double> measured;

    double torque = 0.0;
    const auto steps = static_cast<int>((profile.duration() + 1.5) / dt);
    for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i) * dt;
        joint.update(torque, dt);
        const auto ref = profile.sample(t);
        torque =
            controller.update(ref.position, joint.position(), dt, ref.velocity, ref.acceleration);
        reference.push_back(ref.position);
        measured.push_back(joint.position());
        csv.row({run_id, t, ref.position, joint.position(), torque});
    }
    return {pidpp::analyze_tracking<double>(reference, measured),
            reference.back() - measured.back()};
}

} // namespace

int main(int argc, char** argv) {
    pidpp::PidConfig<double> config{.kp = 120.0,
                                    .ki = 40.0,
                                    .kd = 20.0,
                                    .output_min = -80.0,
                                    .output_max = 80.0,
                                    .integral_min = -1.0,
                                    .integral_max = 1.0,
                                    .anti_windup = pidpp::AntiWindupStrategy::Clamping,
                                    .derivative_mode = pidpp::DerivativeMode::OnError,
                                    .derivative_filter_tau = 0.005};
    if (!config.validate()) {
        return 1;
    }

    pidpp::TrapezoidalProfile<double> profile;
    if (!profile.plan(0.0, std::numbers::pi / 2.0,
                      {.max_velocity = 2.0, .max_acceleration = 6.0})) {
        return 1;
    }

    example::CsvLog csv{argc, argv, {"run", "t", "reference", "position", "torque"}};

    const RunResult feedback_only = run(config, profile, csv, 0.0);

    auto with_ff = config;
    with_ff.feedforward = {
        .velocity_gain = friction, .acceleration_gain = inertia, .enabled = true};
    const RunResult feedforward = run(with_ff, profile, csv, 1.0);

    example::title("Joint trajectory tracking: feedback vs feedback + feedforward");
    example::print_value("PID only: RMS error", feedback_only.metrics.rms_error, "rad");
    example::print_value("PID only: max error", feedback_only.metrics.max_abs_error, "rad");
    example::print_value("PID + FF: RMS error", feedforward.metrics.rms_error, "rad");
    example::print_value("PID + FF: max error", feedforward.metrics.max_abs_error, "rad");
    example::print_value("PID only: final error", feedback_only.final_error, "rad");
    example::print_value("PID + FF: final error", feedforward.final_error, "rad");
    example::print_value("improvement (RMS)",
                         feedback_only.metrics.rms_error / feedforward.metrics.rms_error, "x");

    bool ok = example::check(feedforward.metrics.rms_error * 5.0 < feedback_only.metrics.rms_error,
                             "feedforward cuts RMS tracking error by more than 5x");
    // Without feedforward the integrator charges up while lagging the moving
    // reference and needs time to bleed off after the move: another cost of
    // feedback-only tracking.
    ok &= example::check(std::abs(feedback_only.final_error) < 5.0e-3 &&
                             std::abs(feedforward.final_error) < 1.0e-3,
                         "both runs settle on target");
    return example::finish(ok);
}
