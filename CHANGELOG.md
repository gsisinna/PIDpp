# Changelog

All notable changes to PID++ Control Engine are documented in this file. The project follows
[Semantic Versioning](https://semver.org/); before 1.0, minor versions may change the source API.

## 0.2.0 - 2026-10-25

### Added

- `PidConfig`: `error_deadband` (continuous deadband), `output_rate_limit` (slew limit on the
  command), `continuous_input` with `input_range_min/max` for angles and other wrapping inputs,
  `gains()` / `set_gains()`.
- `PidController`: `update_with_feedforward()` for model-based feedforward terms,
  `bumpless_transfer()`, `set_gains(PidGains)`.
- `FeedforwardConfig::friction_gain` (Coulomb friction, kS).
- `PidGains<T>` value type.
- `CascadedPidController<T>` and `CascadedPidConfig<T>` with outer-loop decimation.
- `GainScheduledPidController<T, N>` with fixed-capacity, linearly interpolated gain tables.
- `RelayAutotuner` limit-cycle analysis (ultimate gain and period, convergence check, timeout)
  and `compute_tuning()` with Ziegler-Nichols, Tyreus-Luyben, Pessen, and overshoot-limited
  rules.
- `TrapezoidalProfile<T>` and `MotionConstraints<T>`.
- `SlewRateLimiter<T>`, `apply_deadband()`, and angle helpers (`wrap_to_range`, `wrap_angle`,
  `angle_difference`, `deg_to_rad`, `rad_to_deg`).
- Step metrics: `StepResponseOptions` (initial value), steady-state error, IAE, ISE, `rose` flag.
  New `analyze_tracking()` for trajectory tracking.
- Simulation models: `SingleLinkArm`, `DifferentialDrive`, `VerticalThrustPlant`, and
  `DcMotor::steady_state_speed()`.
- Eight new self-checking robotics examples (cascaded servo, trajectory tracking, arm gravity
  compensation, payload gain scheduling, mobile robot navigation, drone altitude hold, relay
  autotune, sensor noise filtering); all examples run in CTest and accept `--csv <file>`.
- Control Lab: arm and drone presets, model feedforward, deadband, rate limit, sensor lag,
  dead time, noise, disturbance, relay autotune, P/I/D/FF plot, CSV export, `--headless` mode.
- Doxygen target (`PIDPP_BUILD_DOCS=ON`), CMake check that `version.hpp` matches the project
  version, `.gitattributes` for LF line endings.
- CI: MSVC, AppleClang, install + `find_package` consumer, pkg-config, GUI build and headless
  run, benchmarks, Doxygen; enforcing clang-tidy.

### Changed

- `AntiWindupStrategy::Clamping` now performs dynamic integrator clamping against the output
  limits. It previously behaved exactly like `None`.
- Changing `ki` at runtime rescales the integrator so the integral contribution stays
  continuous (bumpless gain changes).
- `analyze_step_response()` measures overshoot, rise time, and the settling band relative to
  the step size, so steps down and steps from a non-zero initial value are handled.
- Simulation `update()` functions are no longer `[[nodiscard]]`.
- `pidpp::experimental` cascaded, gain-scheduled, and relay headers are now aliases of the
  stable types. `experimental::GainSchedulePoint` now stores `PidGains` instead of a full
  `PidConfig`.

### Fixed

- The Control Lab source was corrupted by a merge (duplicated blocks, unbalanced braces) and did
  not compile; it also broke the clang-format CI job.
- `DcMotor` diverged when `dt` exceeded the electrical time constant; it now sub-steps.
- Non-finite feedforward inputs were not rejected by `PidController::update()`.
- `analyze_step_response()` settling search was O(n^2); it is now O(n).
- Tests failed to link on MSVC and AppleClang because Catch2 was built as C++14.
- Package metadata (Conan, vcpkg) pointed to a non-existent repository URL.

## 0.1.0

### Added

- Initial C++20 PID++ SDK structure.
- PID controller with anti-windup, derivative modes, optional derivative filtering,
  feedforward, and state inspection.
- Core validation, result, version, filter, simulation, autotune, and metrics modules.
- CMake package export, pkg-config generation, examples, tests, and CI scaffolding.
- Optional PID++ Control Lab GUI for interactive simulation and visualization.
