# Architecture

PID++ is a small, deterministic C++20 SDK. The public namespace is `pidpp`, the installable CMake
target is `pidpp::pidpp`, and the facade header is `pidpp/pidpp.hpp`.

## Module map

```
include/pidpp/
  core/          numeric helpers, Result/ErrorCode, dt validation, saturation, deadband, angles
  controllers/   PidConfig, PidController, feedforward, anti-windup and derivative policies,
                 CascadedPidController, GainScheduledPidController
  filters/       LowPassFilter, MovingAverage, SlewRateLimiter
  motion/        TrapezoidalProfile
  autotune/      RelayAutotuner, tuning rules
  metrics/       step-response and tracking metrics (offline)
  simulation/    plant models for examples, tests, and the Control Lab
  experimental/  source-unstable aliases and placeholders (not in the facade)
  internal/      implementation details (not public API)
src/version.cpp  compiled version symbol
tools/gui/       optional Control Lab (simulation core + ImGui front-end)
```

Dependencies point downward: `core` depends on nothing else; `filters` and `motion` depend on
`core`; `controllers` depends on `core` and `filters`; `autotune` depends on `controllers`;
`metrics` and `simulation` depend only on `core`.

## Controller data flow

One `PidController<T>::update()` call:

```
setpoint, measurement
   -> error = setpoint - measurement
   -> wrap to half the input range           (continuous_input)
   -> continuous deadband                    (error_deadband)
   -> P = kp * error
   -> D = kd * filtered derivative           (OnError or OnMeasurement, derivative_filter_tau)
   -> FF = FeedforwardConfig(v, a) or the external term from update_with_feedforward()
   -> output bounds = output limits, narrowed by output_rate_limit * dt
   -> candidate integral, then the anti-windup policy decides the new integral
   -> output = clamp(P + ki * integral + D + FF, bounds)
```

All intermediate terms are stored in `PidState<T>` for logging and diagnostics.

## Real-time constraints

`update()` must be deterministic and allocation-free. It is `noexcept`, keeps all mutable state
inside the object, uses no allocating containers, and performs no virtual dispatch. Invalid
inputs are rejected defensively by returning the previous output.

Configuration validation is explicit and should happen before a controller enters a real-time
loop. Runtime gain updates are supported and bumpless, but callers are responsible for applying
validated values at controlled synchronization points.

## Design trade-offs

**Header-first.** Robotics users often need transparent templates, inlining, and easy
cross-compilation. The library is not purely header-only: `version()` is compiled and the
project installs a normal CMake target, leaving room for non-template code.

**Composition over inheritance.** `CascadedPidController` owns two `PidController`s and
`GainScheduledPidController` owns one plus a table. No virtual interfaces sit in hot paths.

**Values, not exceptions.** `validate()` returns `Result` (an error code plus a static message).
The hot path never throws and never returns errors; it reports invalid input through
`state().last_dt_valid`.

**Fixed capacity.** Gain schedules use `std::array` storage sized by a template parameter;
moving averages use a compile-time window. Nothing in the controllers allocates.

**Integrator units.** The integrator stores accumulated error (error * seconds) and the integral
contribution is `ki * integral`. When `ki` changes, the integrator is rescaled so the
contribution stays continuous.

## Control Lab

`tools/gui` is split into `lab_simulation` (plants, transport delay, sensor lag, noise,
autotune, CSV export; no GUI dependencies) and `main.cpp` (ImGui/ImPlot front-end). The
simulation core runs headless in CI.

## Extension ideas

- Two-degree-of-freedom PID with setpoint weighting.
- Biquad and notch filters for structural resonances.
- S-curve (jerk-limited) motion profiles.
- Lightweight MPC with explicit problem-size templates and fixed workspaces, kept out of the PID
  core.
