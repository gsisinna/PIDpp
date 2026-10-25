# PID++ Control Engine

[![ci](https://github.com/gsisinna/PIDpp/actions/workflows/ci.yml/badge.svg)](https://github.com/gsisinna/PIDpp/actions/workflows/ci.yml)
![C++20](https://img.shields.io/badge/C%2B%2B-20-blue)
![License: MIT](https://img.shields.io/badge/license-MIT-green)

PID++ is a small C++20 library for deterministic PID control in robotics and embedded systems:
motor drives, robot joints, mobile robots, drones, and test rigs. It focuses on the parts that
make a PID loop work on real hardware: saturation and anti-windup, derivative filtering,
feedforward, cascades, gain scheduling, motion profiles, and autotuning.

The repository, headers, namespace, and CMake target use the ASCII-safe name `pidpp`.

```cpp
#include <pidpp/pidpp.hpp>

pidpp::PidConfig<double> config{.kp = 1.0, .ki = 4.0,
                                .output_min = -12.0, .output_max = 12.0,
                                .anti_windup = pidpp::AntiWindupStrategy::Clamping};
if (!config.validate()) { /* reject configuration */ }

pidpp::PidController<double> pid{config};
double volts = pid.update(target_speed, measured_speed, dt);  // noexcept, no allocation
```

## Contents

- [Features](#features)
- [Real-time guarantees](#real-time-guarantees)
- [Quick start](#quick-start)
- [Robotics examples](#robotics-examples)
- [API tour](#api-tour)
- [PID++ Control Lab](#pid-control-lab)
- [Using PID++ in your project](#using-pid-in-your-project)
- [Build options](#build-options)
- [Performance](#performance)
- [Documentation](#documentation)
- [Project status](#project-status)

## Features

**Controllers**

- `PidController<T>`: parallel PID with output limits, four anti-windup strategies (none,
  dynamic clamping, conditional integration, back-calculation), derivative on error or on
  measurement, first-order derivative filter, error deadband, output slew-rate limit,
  continuous (wrapping) inputs for angles, bumpless gain changes, and bumpless transfer.
- Feedforward: reference velocity (kV), acceleration (kA), Coulomb friction (kS), and
  constant load (kG), or any model-based term via `update_with_feedforward()`.
- `CascadedPidController<T>`: position/velocity (or attitude/rate) cascade with outer-loop
  decimation.
- `GainScheduledPidController<T, N>`: gains interpolated from a fixed-capacity table.

**Tuning and analysis**

- `RelayAutotuner<T>`: Astrom-Hagglund relay experiment that estimates the ultimate gain and
  period, plus Ziegler-Nichols, Tyreus-Luyben, Pessen, and overshoot-limited tuning rules.
- Step-response metrics (overshoot, rise and settling time, steady-state error, IAE, ISE)
  and trajectory-tracking metrics (RMS, max, and mean error).

**Signals and motion**

- `TrapezoidalProfile<T>` motion planner, `SlewRateLimiter<T>`, `LowPassFilter<T>`,
  `MovingAverage<T, N>`, and angle helpers (`wrap_angle`, `angle_difference`, ...).

**Simulation models** for examples, tests, and experiments: first-order process, brushed DC
motor, mass-spring-damper joint, single-link arm with gravity and payload, differential-drive
robot, and multirotor vertical axis.

All controllers are templates over `float` or `double`, so the same code runs on a desktop
simulator or an FPU-equipped microcontroller.

## Real-time guarantees

For `PidController<T>::update()` and the other controller `update()` functions:

- `noexcept`, no heap allocation, no logging, no virtual dispatch;
- deterministic O(1) work (O(N) table lookup for gain scheduling);
- invalid `dt`, setpoint, measurement, or feedforward input (NaN, infinity, `dt <= 0`) keeps
  the previous output and sets `state().last_dt_valid = false`.

Configuration is validated explicitly with `validate()`, which returns a `Result` instead of
throwing. Validation, metrics, autotune analysis, and simulation helpers are intended to run
outside hard real-time loops unless documented otherwise. See
[docs/realtime_contract.md](docs/realtime_contract.md).

## Quick start

Requirements: CMake 3.20+ and a C++20 compiler (GCC 11+, Clang 14+, MSVC 2022, AppleClang 15+).

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

`ctest` runs the unit tests and every example scenario. Run an example directly:

```sh
./build/examples/cascaded_servo_control
./build/examples/drone_altitude_hold --csv altitude.csv   # also export the time series
```

## Robotics examples

Each example is a small, commented program that simulates a realistic scenario, prints a
report, and checks its own acceptance criteria (non-zero exit on failure). Details and
discussion are in [docs/examples.md](docs/examples.md).

| Example | Scenario | Shows |
| --- | --- | --- |
| [`motor_speed_control`](examples/motor_speed_control.cpp) | Wheel/conveyor speed loop with a load step | PI + kV feedforward, conditional integration |
| [`joint_position_control`](examples/joint_position_control.cpp) | 90 deg joint step | Derivative on measurement, back-calculation |
| [`cascaded_servo_control`](examples/cascaded_servo_control.cpp) | Servo drive, one-revolution move | Position/velocity cascade, profile feedforward, decimation |
| [`trajectory_tracking`](examples/trajectory_tracking.cpp) | Pick-and-place joint move | PID vs PID + kV/kA feedforward (about 800x lower RMS error) |
| [`arm_gravity_compensation`](examples/arm_gravity_compensation.cpp) | Shoulder joint lifting a link, payload pickup | `update_with_feedforward()` with `m*g*l*cos(q)` |
| [`gain_scheduling_payload`](examples/gain_scheduling_payload.cpp) | SCARA joint with 0 to 4 kg payload | Gain scheduling on payload mass |
| [`mobile_robot_navigation`](examples/mobile_robot_navigation.cpp) | Differential-drive robot | Continuous heading input, waypoint following, slew limiting |
| [`drone_altitude_hold`](examples/drone_altitude_hold.cpp) | Quadrotor take-off, hover, package drop | Hover feedforward, thrust limits, output rate limit |
| [`relay_autotune`](examples/relay_autotune.cpp) | Motor speed loop autotuning | Relay experiment, tuning rules, closed-loop check |
| [`sensor_noise_filtering`](examples/sensor_noise_filtering.cpp) | Joint with a quantized, noisy encoder | Derivative filter vs actuator chatter |
| [`step_response_analysis`](examples/step_response_analysis.cpp) | P vs PI vs PID on a lag process | Step metrics |
| [`basic_pid`](examples/basic_pid.cpp) | One update | Minimal API |

## API tour

### Configure, validate, run

```cpp
pidpp::PidConfig<double> config{
    .kp = 45.0, .ki = 2.0, .kd = 8.0,
    .output_min = -60.0, .output_max = 60.0,       // actuator limits (N*m)
    .integral_min = -5.0, .integral_max = 5.0,     // accumulated error bounds (rad*s)
    .anti_windup = pidpp::AntiWindupStrategy::BackCalculation,
    .derivative_mode = pidpp::DerivativeMode::OnMeasurement,  // no setpoint kick
    .derivative_filter_tau = 0.02};                // s

if (const auto result = config.validate(); !result) {
    std::fprintf(stderr, "%s\n", result.message());
}

pidpp::PidController<double> pid{config};
const double torque = pid.update(target, position, dt);
const auto& s = pid.state();  // P, I, D, FF terms, saturation flag, ...
```

### Feedforward along a motion profile

```cpp
pidpp::TrapezoidalProfile<double> profile;
profile.plan(start, goal, {.max_velocity = 2.0, .max_acceleration = 6.0});

config.feedforward = {.velocity_gain = viscous_friction, .acceleration_gain = inertia,
                      .enabled = true};
pidpp::PidController<double> pid{config};

const auto ref = profile.sample(t);
torque = pid.update(ref.position, position, dt, ref.velocity, ref.acceleration);
```

### Model-based feedforward (gravity compensation)

```cpp
const double gravity = mass * g * com_distance * std::cos(angle);
torque = pid.update_with_feedforward(target, angle, dt, gravity);
```

### Cascaded servo

```cpp
pidpp::CascadedPidConfig<double> servo_config{};
servo_config.outer = {.kp = 15.0, .output_min = -60.0, .output_max = 60.0,  // rad/s limit
                      .feedforward = {.velocity_gain = 1.0, .enabled = true}};
servo_config.inner = {.kp = 4.0, .ki = 60.0, .output_min = -24.0, .output_max = 24.0,
                      .integral_min = -5.0, .integral_max = 5.0,
                      .anti_windup = pidpp::AntiWindupStrategy::Clamping};
servo_config.outer_decimation = 2;  // position loop at half the velocity loop rate

pidpp::CascadedPidController<double> servo{servo_config};
volts = servo.update(ref.position, position, velocity, dt, ref.velocity, ref.acceleration);
```

### Angles and heading control

```cpp
config.continuous_input = true;  // default range [-pi, pi)
// From +170 deg to -170 deg the controller turns +20 deg, not -340 deg.
yaw_rate = heading_pid.update(target_heading, robot_heading, dt);
```

### Gain scheduling

```cpp
pidpp::GainScheduledPidController<double, 8> pid{base_config};
const std::array<pidpp::GainSchedulePoint<double>, 3> table{{
    {0.0, {.kp = 20.0, .ki = 2.0, .kd = 3.0}},   // empty gripper
    {2.0, {.kp = 50.0, .ki = 5.0, .kd = 7.5}},   // 2 kg payload
    {4.0, {.kp = 80.0, .ki = 8.0, .kd = 12.0}},  // 4 kg payload
}};
if (!pid.set_schedule(table)) { /* unsorted or invalid table */ }
torque = pid.update(payload_kg, target, angle, dt);
```

### Relay autotuning

```cpp
pidpp::RelayAutotuner<double> tuner{{.relay_amplitude = 2.0, .hysteresis = 0.2,
                                     .output_bias = 6.0, .timeout = 20.0}};
while (!tuner.complete() && !tuner.result().timed_out) {
    voltage = tuner.update(target_speed, read_speed(), dt);
    apply(voltage);
}
config.set_gains(tuner.gains(pidpp::TuningRule::TyreusLuybenPi));
```

## PID++ Control Lab

An optional desktop tool (Dear ImGui + ImPlot) for interactive tuning. Pick a plant (process
with dead time, DC motor, joint, arm with gravity, drone), change gains and policies, add
noise, sensor lag, dead time, or a disturbance, run the relay autotuner, and inspect the
response, the actuator command, and each P/I/D/FF contribution. Results can be exported to CSV.

```sh
# Ubuntu/Debian prerequisites
sudo apt-get install libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev

cmake -S . -B build-gui -DPIDPP_BUILD_GUI=ON
cmake --build build-gui --target pidpp_control_lab
./build-gui/tools/gui/pidpp_control_lab
```

`pidpp_control_lab --headless` runs every preset without a window; CI uses it as a smoke test.

## Using PID++ in your project

### Installed package

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPIDPP_BUILD_TESTS=OFF -DPIDPP_BUILD_EXAMPLES=OFF
cmake --build build
cmake --install build --prefix /opt/pidpp
```

```cmake
find_package(PIDPP 0.2 CONFIG REQUIRED)
target_link_libraries(my_robot PRIVATE pidpp::pidpp)
```

Configure your project with `-DCMAKE_PREFIX_PATH=/opt/pidpp`. A `pidpp.pc` file is installed
for pkg-config users.

### FetchContent

```cmake
include(FetchContent)
FetchContent_Declare(pidpp GIT_REPOSITORY https://github.com/gsisinna/PIDpp.git GIT_TAG main)
set(PIDPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(PIDPP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(pidpp)
target_link_libraries(my_robot PRIVATE pidpp::pidpp)
```

More in [docs/cmake_integration.md](docs/cmake_integration.md). Conan and vcpkg manifests are
included (`conanfile.py`, `vcpkg.json`).

## Build options

| Option | Default | Description |
| --- | --- | --- |
| `PIDPP_BUILD_TESTS` | `ON` | Unit tests (Catch2, fetched) and example scenarios in CTest |
| `PIDPP_BUILD_EXAMPLES` | `ON` | Robotics examples |
| `PIDPP_BUILD_BENCHMARKS` | `OFF` | Google Benchmark latency benchmarks |
| `PIDPP_BUILD_GUI` | `OFF` | PID++ Control Lab (GLFW, Dear ImGui, ImPlot, fetched) |
| `PIDPP_BUILD_DOCS` | `OFF` | `pidpp_docs` Doxygen target |
| `PIDPP_ENABLE_WARNINGS` | `ON` | Strict warnings on PID++ targets |
| `PIDPP_ENABLE_SANITIZERS` | `OFF` | AddressSanitizer + UBSan on PID++ targets (GCC/Clang) |
| `PIDPP_INSTALL` | `ON` | Install targets, headers, CMake package, and pkg-config file |
| `PIDPP_EMBEDDED_PROFILE` | `OFF` | Embedded-friendly defaults (no tests, examples, benchmarks, docs) |

## Performance

`pidpp_update_benchmark` on a desktop x86-64 CPU (GCC 11, `-O3`), per call:

| Benchmark | Time |
| --- | --- |
| PID, no derivative filter | ~1.2 ns |
| PID with derivative filter | ~3.0 ns |
| PI with dynamic clamping | ~3.7 ns |
| Cascaded position/velocity | ~16 ns |
| Gain scheduled (4 points) | ~6.4 ns |
| Trapezoidal profile sample | ~1.5 ns |

## Documentation

- [Tuning guide](docs/tuning_guide.md): a practical procedure for robotics loops.
- [Examples](docs/examples.md): what each example demonstrates and why.
- [Architecture](docs/architecture.md) and [API design](docs/api_design.md).
- [Real-time contract](docs/realtime_contract.md).
- [CMake integration](docs/cmake_integration.md).
- API reference: `cmake -S . -B build -DPIDPP_BUILD_DOCS=ON && cmake --build build --target pidpp_docs`
  (also uploaded as a CI artifact).

## Project status

PID++ is at version 0.2.0. Before 1.0 the public API may still change between minor versions;
see [CHANGELOG.md](CHANGELOG.md). APIs under `pidpp::experimental` may change at any time and
headers under `include/pidpp/internal` are implementation details. ABI stability is not
promised before 1.0.

Ideas on the roadmap: discrete-time setpoint weighting (2-DOF PID), notch and biquad filters,
S-curve motion profiles, and a lightweight MPC module with fixed-size workspaces.

Contributions are welcome; see [CONTRIBUTING.md](CONTRIBUTING.md).

## License

MIT. See [LICENSE](LICENSE).
