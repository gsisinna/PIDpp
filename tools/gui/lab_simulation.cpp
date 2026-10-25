#include "lab_simulation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <vector>

namespace lab {

namespace {

/// Deterministic uniform noise in [-1, 1] (identical on every platform).
class Noise {
  public:
    double next() noexcept {
        state_ = (state_ * 6364136223846793005ULL) + 1442695040888963407ULL;
        const auto bits = static_cast<double>(state_ >> 11U) * 0x1.0p-53;
        return (2.0 * bits) - 1.0;
    }

  private:
    std::uint64_t state_{0x5EEDULL};
};

/// Pure transport delay on the actuator path, as a ring buffer of samples.
class TransportDelay {
  public:
    TransportDelay(double delay, double dt, double initial)
        : buffer_(static_cast<std::size_t>(std::max(0.0, std::round(delay / dt))), initial) {}

    double push(double value) {
        if (buffer_.empty()) {
            return value;
        }
        const double delayed = buffer_[index_];
        buffer_[index_] = value;
        index_ = (index_ + 1U) % buffer_.size();
        return delayed;
    }

  private:
    std::vector<double> buffer_;
    std::size_t index_{0};
};

/// Wraps the SDK plant models behind one interface for the lab.
class Plant {
  public:
    Plant(PlantModel model, double initial) : model_{model} {
        switch (model_) {
        case PlantModel::FirstOrder:
            first_order_.reset(initial);
            break;
        case PlantModel::DcMotorVelocity:
            break;
        case PlantModel::JointPosition:
            joint_.reset(initial, 0.0);
            break;
        case PlantModel::ArmWithGravity:
            arm_.reset(initial, 0.0);
            break;
        case PlantModel::DroneAltitude:
            drone_.reset(std::max(initial, 0.0), 0.0);
            break;
        case PlantModel::Count:
            break;
        }
    }

    /// Advances the plant by dt with actuator input `u`; returns the true output.
    double step(double u, double dt) {
        switch (model_) {
        case PlantModel::FirstOrder:
            return first_order_.update(u, dt);
        case PlantModel::DcMotorVelocity:
            return motor_.update(u, 0.0, dt);
        case PlantModel::JointPosition:
            return joint_.update(u, dt);
        case PlantModel::ArmWithGravity:
            return arm_.update(u, dt);
        case PlantModel::DroneAltitude:
            return drone_.update(u, dt);
        case PlantModel::Count:
            break;
        }
        return 0.0;
    }

    /// Model-based feedforward: the input that holds the plant at `setpoint`.
    [[nodiscard]] double feedforward(double setpoint, double measurement) const {
        switch (model_) {
        case PlantModel::FirstOrder:
            return setpoint / first_order_gain;
        case PlantModel::DcMotorVelocity:
            return setpoint / motor_.steady_state_speed(1.0);
        case PlantModel::JointPosition:
            return 0.0;
        case PlantModel::ArmWithGravity:
            return arm_.gravity_torque(measurement);
        case PlantModel::DroneAltitude:
            return drone_.hover_thrust();
        case PlantModel::Count:
            break;
        }
        return 0.0;
    }

  private:
    static constexpr double first_order_gain = 1.0;

    PlantModel model_;
    pidpp::FirstOrderPlant<double> first_order_{first_order_gain, 0.35};
    pidpp::DcMotor<double> motor_{2.0, 0.01, 0.1, 0.1, 0.01, 0.001};
    pidpp::MassSpringDamper<double> joint_{1.2, 1.4, 0.0};
    pidpp::SingleLinkArm<double> arm_{2.0, 0.6, 0.8, 0.05};
    pidpp::VerticalThrustPlant<double> drone_{1.2, 0.4, 0.04};
};

[[nodiscard]] PlantModel model_of(const Settings& settings) {
    const int index = std::clamp(settings.plant, 0, static_cast<int>(PlantModel::Count) - 1);
    return static_cast<PlantModel>(index);
}

[[nodiscard]] bool valid_timing(const Settings& settings, double& dt, int& steps,
                                std::string& status) {
    dt = static_cast<double>(settings.dt_ms) / 1000.0;
    if (!pidpp::validate_time_step(dt) || settings.duration_s <= 0.0F) {
        status = "Invalid simulation step or duration";
        return false;
    }
    steps = static_cast<int>(std::floor(static_cast<double>(settings.duration_s) / dt));
    steps = std::clamp(steps, 1, 2'000'000);
    return true;
}

} // namespace

const char* plant_name(PlantModel model) noexcept {
    switch (model) {
    case PlantModel::FirstOrder:
        return "First-order process";
    case PlantModel::DcMotorVelocity:
        return "DC motor velocity";
    case PlantModel::JointPosition:
        return "Joint position (no gravity)";
    case PlantModel::ArmWithGravity:
        return "Robot arm with gravity";
    case PlantModel::DroneAltitude:
        return "Drone altitude";
    case PlantModel::Count:
        break;
    }
    return "Unknown";
}

const char* plant_combo_items() noexcept {
    return "First-order process\0DC motor velocity\0Joint position (no gravity)\0"
           "Robot arm with gravity\0Drone altitude\0";
}

Settings defaults_for(PlantModel model) {
    Settings s{};
    s.plant = static_cast<int>(model);
    switch (model) {
    case PlantModel::FirstOrder:
        s.kp = 0.8F;
        s.ki = 2.0F;
        s.kd = 0.02F;
        s.dead_time_ms = 50.0F;
        s.setpoint = 10.0F;
        s.duration_s = 4.0F;
        break;
    case PlantModel::DcMotorVelocity:
        s.kp = 1.0F;
        s.ki = 4.0F;
        s.kd = 0.0F;
        s.integral_min = -20.0F;
        s.integral_max = 20.0F;
        s.setpoint = 50.0F;
        s.duration_s = 3.0F;
        s.model_feedforward = true;
        s.disturbance_time = 1.5F;
        s.disturbance = -1.0F;
        break;
    case PlantModel::JointPosition:
        s.kp = 45.0F;
        s.ki = 2.0F;
        s.kd = 8.0F;
        s.output_min = -60.0F;
        s.output_max = 60.0F;
        s.integral_min = -5.0F;
        s.integral_max = 5.0F;
        s.anti_windup = static_cast<int>(pidpp::AntiWindupStrategy::BackCalculation);
        s.setpoint = 1.57F;
        s.duration_s = 5.0F;
        break;
    case PlantModel::ArmWithGravity:
        s.kp = 60.0F;
        s.ki = 80.0F;
        s.kd = 8.0F;
        s.output_min = -40.0F;
        s.output_max = 40.0F;
        s.integral_min = -1.0F;
        s.integral_max = 1.0F;
        s.derivative_filter_tau = 0.005F;
        s.anti_windup = static_cast<int>(pidpp::AntiWindupStrategy::Clamping);
        s.model_feedforward = true;
        s.initial_measurement = -1.57F;
        s.setpoint = 0.5F;
        s.duration_s = 6.0F;
        s.disturbance_time = 3.0F;
        s.disturbance = -2.5F;
        break;
    case PlantModel::DroneAltitude:
        s.kp = 10.0F;
        s.ki = 2.0F;
        s.kd = 3.0F;
        s.output_min = 0.0F;
        s.output_max = 25.0F;
        s.integral_min = -5.0F;
        s.integral_max = 5.0F;
        s.output_rate_limit = 80.0F;
        s.model_feedforward = true;
        s.setpoint = 10.0F;
        s.dt_ms = 2.0F;
        s.duration_s = 20.0F;
        s.disturbance_time = 12.0F;
        s.disturbance = 3.0F;
        break;
    case PlantModel::Count:
        break;
    }
    return s;
}

void Trace::clear(std::size_t reserve) {
    for (auto* series :
         {&time, &setpoint, &measurement, &command, &p_term, &i_term, &d_term, &ff_term}) {
        series->clear();
        series->reserve(reserve);
    }
    metrics = {};
    valid = false;
}

pidpp::PidConfig<double> make_config(const Settings& settings) {
    pidpp::PidConfig<double> config{};
    config.kp = static_cast<double>(settings.kp);
    config.ki = static_cast<double>(settings.ki);
    config.kd = static_cast<double>(settings.kd);
    config.output_min = static_cast<double>(settings.output_min);
    config.output_max = static_cast<double>(settings.output_max);
    config.integral_min = static_cast<double>(settings.integral_min);
    config.integral_max = static_cast<double>(settings.integral_max);
    config.anti_windup =
        static_cast<pidpp::AntiWindupStrategy>(std::clamp(settings.anti_windup, 0, 3));
    config.derivative_mode =
        static_cast<pidpp::DerivativeMode>(std::clamp(settings.derivative_mode, 0, 1));
    config.derivative_filter_tau = static_cast<double>(settings.derivative_filter_tau);
    config.back_calculation_gain = static_cast<double>(settings.back_calculation_gain);
    config.error_deadband = static_cast<double>(settings.error_deadband);
    config.output_rate_limit = static_cast<double>(settings.output_rate_limit);
    return config;
}

void run_simulation(const Settings& settings, Trace& trace) {
    const auto config = make_config(settings);
    if (const auto validation = config.validate(); !validation) {
        trace.valid = false;
        trace.status = std::string{"Invalid configuration: "} + validation.message();
        return;
    }

    double dt = 0.0;
    int steps = 0;
    if (!valid_timing(settings, dt, steps, trace.status)) {
        trace.valid = false;
        return;
    }

    trace.clear(static_cast<std::size_t>(steps));
    const PlantModel model = model_of(settings);
    Plant plant{model, static_cast<double>(settings.initial_measurement)};
    pidpp::PidController<double> controller{config};
    Noise noise;
    pidpp::LowPassFilter<double> sensor{static_cast<double>(settings.sensor_lag_ms) / 1000.0};
    TransportDelay delay{static_cast<double>(settings.dead_time_ms) / 1000.0, dt, 0.0};

    const auto setpoint = static_cast<double>(settings.setpoint);
    const auto noise_amplitude = static_cast<double>(settings.noise_amplitude);
    double command = 0.0;
    for (int i = 0; i < steps; ++i) {
        const double t = static_cast<double>(i) * dt;
        const double disturbance = t >= static_cast<double>(settings.disturbance_time)
                                       ? static_cast<double>(settings.disturbance)
                                       : 0.0;
        const double output = plant.step(delay.push(command) + disturbance, dt);
        const double measured = sensor.update(output + (noise_amplitude * noise.next()), dt);
        const double feedforward =
            settings.model_feedforward ? plant.feedforward(setpoint, measured) : 0.0;
        command = controller.update_with_feedforward(setpoint, measured, dt, feedforward);

        const auto& state = controller.state();
        trace.time.push_back(t);
        trace.setpoint.push_back(setpoint);
        trace.measurement.push_back(output);
        trace.command.push_back(command);
        trace.p_term.push_back(state.proportional_term);
        trace.i_term.push_back(state.integral_term);
        trace.d_term.push_back(state.derivative_term);
        trace.ff_term.push_back(state.feedforward_term);
    }

    trace.metrics = pidpp::analyze_step_response<double>(
        trace.time, trace.measurement,
        pidpp::StepResponseOptions<double>{.initial_value =
                                               static_cast<double>(settings.initial_measurement),
                                           .target = setpoint});
    trace.valid = std::isfinite(trace.measurement.back());
    trace.status = trace.valid ? std::string{"Simulated "} + plant_name(model)
                               : std::string{"Simulation diverged: reduce gains or dt"};
}

AutotuneOutcome autotune(const Settings& settings) {
    AutotuneOutcome outcome;
    double dt = 0.0;
    int steps = 0;
    if (!valid_timing(settings, dt, steps, outcome.message)) {
        return outcome;
    }
    const auto config = make_config(settings);
    if (!config.validate()) {
        outcome.message = "Fix the output limits before autotuning";
        return outcome;
    }

    const PlantModel model = model_of(settings);
    Plant plant{model, static_cast<double>(settings.initial_measurement)};
    const auto setpoint = static_cast<double>(settings.setpoint);
    const double bias =
        std::clamp(plant.feedforward(setpoint, setpoint), config.output_min, config.output_max);
    const double headroom = std::min(config.output_max - bias, bias - config.output_min);
    if (headroom <= 0.0) {
        outcome.message = "No output headroom around the operating point";
        return outcome;
    }

    pidpp::RelayAutotuner<double> tuner{
        {.relay_amplitude = 0.5 * headroom,
         .hysteresis = 2.0 * static_cast<double>(settings.noise_amplitude),
         .output_bias = bias,
         .cycles = 4,
         .timeout = std::max(60.0, 10.0 * static_cast<double>(settings.duration_s))}};
    if (const auto validation = tuner.config().validate(); !validation) {
        outcome.message = std::string{"Relay setup invalid: "} + validation.message();
        return outcome;
    }

    Noise noise;
    pidpp::LowPassFilter<double> sensor{static_cast<double>(settings.sensor_lag_ms) / 1000.0};
    TransportDelay delay{static_cast<double>(settings.dead_time_ms) / 1000.0, dt, bias};
    double command = bias;
    const auto max_steps = static_cast<int>(tuner.config().timeout / dt) + 1;
    for (int i = 0; i < max_steps && !tuner.complete() && !tuner.result().timed_out; ++i) {
        const double raw = plant.step(delay.push(command), dt) +
                           (static_cast<double>(settings.noise_amplitude) * noise.next());
        const double measured = sensor.update(raw, dt);
        command = tuner.update(setpoint, measured, dt);
    }

    outcome.result = tuner.result();
    if (!tuner.complete()) {
        outcome.message = "Relay experiment did not reach a stable limit cycle";
        return outcome;
    }
    const auto rule = static_cast<pidpp::TuningRule>(
        std::clamp(settings.tuning_rule, 0, static_cast<int>(pidpp::TuningRule::NoOvershootPid)));
    outcome.gains = tuner.gains(rule);
    outcome.success = static_cast<bool>(outcome.gains.validate());
    outcome.message = outcome.success ? "Autotune complete" : "Autotune produced invalid gains";
    return outcome;
}

bool export_csv(const Trace& trace, const std::string& path) {
    std::ofstream file{path};
    if (!file) {
        return false;
    }
    file << "t,setpoint,measurement,command,p,i,d,ff\n";
    for (std::size_t i = 0; i < trace.time.size(); ++i) {
        file << trace.time[i] << ',' << trace.setpoint[i] << ',' << trace.measurement[i] << ','
             << trace.command[i] << ',' << trace.p_term[i] << ',' << trace.i_term[i] << ','
             << trace.d_term[i] << ',' << trace.ff_term[i] << '\n';
    }
    return static_cast<bool>(file);
}

} // namespace lab
