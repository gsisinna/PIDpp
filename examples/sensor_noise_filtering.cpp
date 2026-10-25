// Derivative filtering with a noisy, quantized encoder.
//
// Derivative action amplifies sensor noise: differentiating a quantized
// encoder signal at 1 kHz produces spikes that reach the motor as torque
// chatter, heat, and audible noise. A first-order derivative filter
// (`derivative_filter_tau`) trades a little phase lag for a much quieter
// actuator command. Both runs use identical gains.
//
// The noise generator is a tiny deterministic LCG so results are identical
// on every platform.

#include <cmath>
#include <cstdint>
#include <numbers>

#include <pidpp/pidpp.hpp>

#include "common/example_support.hpp"

namespace {

constexpr double sample_time = 0.001;
constexpr double counts_per_rev = 4096.0; // 12-bit absolute encoder
constexpr double target_angle = std::numbers::pi / 4.0;

class Encoder {
  public:
    double read(double angle) {
        const double resolution = 2.0 * std::numbers::pi / counts_per_rev;
        const double noisy = angle + (uniform() - 0.5) * 2.0 * resolution;
        return std::round(noisy / resolution) * resolution;
    }

  private:
    double uniform() {
        state_ = (state_ * 6364136223846793005ULL) + 1442695040888963407ULL;
        return static_cast<double>(state_ >> 11U) * 0x1.0p-53;
    }

    std::uint64_t state_{42};
};

struct RunResult {
    double command_chatter; // RMS of the command change per sample
    double mean_error;      // mean absolute position error at steady state
};

RunResult run(double derivative_filter_tau, example::CsvLog& csv) {
    pidpp::PidConfig<double> config{.kp = 40.0,
                                    .ki = 40.0,
                                    .kd = 4.0,
                                    .output_min = -30.0,
                                    .output_max = 30.0,
                                    .integral_min = -1.0,
                                    .integral_max = 1.0,
                                    .anti_windup = pidpp::AntiWindupStrategy::Clamping,
                                    .derivative_mode = pidpp::DerivativeMode::OnMeasurement,
                                    .derivative_filter_tau = derivative_filter_tau};
    pidpp::PidController<double> controller{config};
    pidpp::MassSpringDamper<double> joint{0.5, 0.5, 0.0};
    Encoder encoder;

    double torque = 0.0;
    double previous_torque = 0.0;
    double chatter_sum = 0.0;
    double error_sum = 0.0;
    int chatter_samples = 0;
    for (int i = 0; i < 4000; ++i) {
        joint.update(torque, sample_time);
        torque = controller.update(target_angle, encoder.read(joint.position()), sample_time);
        if (i >= 2000) { // steady state only
            const double delta = torque - previous_torque;
            chatter_sum += delta * delta;
            error_sum += std::abs(target_angle - joint.position());
            ++chatter_samples;
        }
        previous_torque = torque;
        csv.row({derivative_filter_tau, i * sample_time, joint.position(), torque});
    }
    return {std::sqrt(chatter_sum / chatter_samples), error_sum / chatter_samples};
}

} // namespace

int main(int argc, char** argv) {
    example::CsvLog csv{argc, argv, {"tau", "t", "position", "torque"}};

    const RunResult raw = run(0.0, csv);
    const RunResult filtered = run(0.01, csv);

    example::title("Encoder noise: unfiltered vs filtered derivative (tau = 10 ms)");
    example::print_value("unfiltered: command chatter", raw.command_chatter, "N m RMS");
    example::print_value("filtered: command chatter", filtered.command_chatter, "N m RMS");
    example::print_value("unfiltered: mean abs error", raw.mean_error, "rad");
    example::print_value("filtered: mean abs error", filtered.mean_error, "rad");

    bool ok = example::check(filtered.command_chatter * 3.0 < raw.command_chatter,
                             "filter reduces command chatter by more than 3x");
    // One encoder count is 1.5 mrad; the loop should hold within a few counts.
    ok &= example::check(filtered.mean_error < 4.0 * 2.0 * std::numbers::pi / counts_per_rev,
                         "filtered loop holds position within 4 encoder counts");
    return example::finish(ok);
}
