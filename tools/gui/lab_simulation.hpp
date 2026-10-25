#pragma once

// Simulation core of the PID++ Control Lab, independent of any GUI code so it
// can run headless (CI smoke test) and stay easy to read.

#include <cstddef>
#include <string>
#include <vector>

#include <pidpp/pidpp.hpp>

namespace lab {

enum class PlantModel : int {
    FirstOrder = 0,
    DcMotorVelocity,
    JointPosition,
    ArmWithGravity,
    DroneAltitude,
    Count
};

[[nodiscard]] const char* plant_name(PlantModel model) noexcept;
/// Null-separated list of plant names for ImGui::Combo.
[[nodiscard]] const char* plant_combo_items() noexcept;

/// Everything the user can change. Plain floats/ints map directly to widgets.
struct Settings {
    float kp{1.2F};
    float ki{0.4F};
    float kd{0.05F};
    float output_min{-12.0F};
    float output_max{12.0F};
    float integral_min{-10.0F};
    float integral_max{10.0F};
    float derivative_filter_tau{0.02F};
    float back_calculation_gain{1.0F};
    float error_deadband{0.0F};
    float output_rate_limit{0.0F};
    int anti_windup{static_cast<int>(pidpp::AntiWindupStrategy::ConditionalIntegration)};
    int derivative_mode{static_cast<int>(pidpp::DerivativeMode::OnMeasurement)};

    int plant{static_cast<int>(PlantModel::FirstOrder)};
    bool model_feedforward{false}; ///< gravity / hover / back-EMF feedforward
    float setpoint{10.0F};
    float initial_measurement{0.0F};
    float dt_ms{1.0F};
    float duration_s{4.0F};
    float noise_amplitude{0.0F};  ///< uniform measurement noise, +/- amplitude
    float sensor_lag_ms{10.0F};   ///< first-order sensor/estimator lag, 0 = ideal sensor
    float dead_time_ms{0.0F};     ///< actuator transport delay
    float disturbance_time{2.0F}; ///< s
    float disturbance{0.0F};      ///< added to the actuator input

    int tuning_rule{static_cast<int>(pidpp::TuningRule::TyreusLuybenPid)};
};

/// Sensible defaults (gains, limits, setpoint) for each plant.
[[nodiscard]] Settings defaults_for(PlantModel model);

struct Trace {
    std::vector<double> time;
    std::vector<double> setpoint;
    std::vector<double> measurement;
    std::vector<double> command;
    std::vector<double> p_term;
    std::vector<double> i_term;
    std::vector<double> d_term;
    std::vector<double> ff_term;
    pidpp::StepResponseMetrics<double> metrics{};
    std::string status{"Ready"};
    bool valid{false};

    void clear(std::size_t reserve);
};

[[nodiscard]] pidpp::PidConfig<double> make_config(const Settings& settings);

/// Runs one closed-loop simulation; on invalid settings `trace.valid` is false.
void run_simulation(const Settings& settings, Trace& trace);

struct AutotuneOutcome {
    bool success{false};
    pidpp::RelayAutotuneResult<double> result{};
    pidpp::PidGains<double> gains{};
    std::string message;
};

/// Runs a relay experiment on the selected plant and computes gains.
[[nodiscard]] AutotuneOutcome autotune(const Settings& settings);

/// Writes the trace as CSV. Returns false on I/O failure.
bool export_csv(const Trace& trace, const std::string& path);

} // namespace lab
