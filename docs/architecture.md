# Architecture

PID++ Control Engine is organized as a small, deterministic C++20 SDK. The public namespace is `pidpp`, the installable CMake target is `pidpp::pidpp`, and the main facade header is `pidpp/pidpp.hpp`.

## Module Boundaries

`core` contains shared primitives: numeric helpers, result reporting, time-step validation, and saturation utilities. These headers have no dependency on controller implementations.

`controllers` contains real-time controller components. `PidConfig<T>` owns validation and tunable parameters. `PidController<T>` owns state and update logic. Anti-windup, derivative mode, and feedforward are separate value-type concepts so future controllers can reuse them without inheriting from a shared base class.

`filters` contains deterministic filters. `LowPassFilter<T>` is a small stateful single-pole filter. `MovingAverage<T, N>` uses `std::array` and a compile-time window size, so it does not allocate.

`autotune` contains relay autotuning interfaces. The current implementation provides deterministic relay switching and leaves richer ultimate gain and period estimation as future work.

`metrics` is offline analysis. It accepts caller-owned spans and may trade runtime cost for clarity because it is not intended for real-time loops.

`simulation` contains lightweight plant models used by examples and tests. These are useful for control design smoke tests, not high-fidelity physics.

`experimental` contains source-unstable extension points. These headers are not included by the facade and should be opted into explicitly.

`internal` contains implementation details. Downstream projects should not include these headers directly.

## Real-Time Constraints

The central rule is that `PidController<T>::update()` must be deterministic and allocation-free. The update path:

- is marked `noexcept`;
- stores all mutable state inside the controller object;
- uses no standard containers that allocate;
- performs no virtual dispatch;
- validates invalid `dt`, setpoint, and measurement values defensively by returning the previous output.

Configuration validation is explicit and should happen before a controller enters a real-time loop. Runtime gain updates are supported, but callers are responsible for applying validated values when operating in safety-critical systems.

## Design Tradeoffs

The library is header-first because robotics users often need transparent templates, inlining, and easy cross-compilation. It is not purely header-only: the facade exposes a compiled `version()` symbol and the project installs a normal CMake target, leaving room for future non-template implementation files.

Composition is preferred over inheritance. This keeps hot paths predictable and leaves room for policy-style extension without forcing virtual interfaces into control loops.

Invalid time steps in `update()` do not throw and do not return an error object because the hot path must stay small and hard to misuse. Instead, the controller preserves the previous output and records `state().last_dt_valid`.

Anti-windup behavior is selected by enum:

- `None`: integrate and clamp only to configured integral bounds.
- `Clamping`: explicit integral clamping.
- `ConditionalIntegration`: only integrate when saturation is not being driven deeper.
- `BackCalculation`: apply a bounded correction from saturated output error.

## Extension Strategy

Cascaded PID can be added as a composed controller that owns two or more `PidController<T>` instances and defines the signal contract between stages.

Gain scheduling should use fixed-size tables or caller-owned spans to preserve deterministic memory behavior. Scheduling policies can produce a `PidConfig<T>` or direct gain triplet without changing the PID update algorithm.

Adaptive control can be introduced as a parameter update policy that runs at a slower supervisory rate. The policy should validate and apply gains outside the tightest update path when possible.

Feedforward can grow from the current velocity, acceleration, and static terms into profile-aware policies. The controller already accepts feedforward inputs without requiring inheritance.

Lightweight MPC should live in its own module with explicit problem-size templates or fixed-capacity workspaces. It should not pull heap allocation or large solver dependencies into the PID core.
