# API Design

PID++ separates public, experimental, and internal API surfaces.

## Stable public API

Everything reachable from `include/pidpp/pidpp.hpp`:

- `include/pidpp/version.hpp`
- `include/pidpp/core`
- `include/pidpp/controllers`
- `include/pidpp/filters`
- `include/pidpp/motion`
- `include/pidpp/autotune`
- `include/pidpp/metrics`
- `include/pidpp/simulation`

## Conventions

- **Configuration structs** (`PidConfig`, `CascadedPidConfig`, `RelayAutotuneConfig`,
  `MotionConstraints`) are aggregates designed for C++20 designated initializers and expose
  `[[nodiscard]] Result validate() const noexcept`.
- **Controllers** are constructed from a configuration, expose `update(...)` returning the
  actuator command, `reset()`, and `state()` for diagnostics.
- **Errors** are reported with `Result` and `ErrorCode`; messages point to static storage.
  Nothing in the library throws.
- **Units** are SI and seconds throughout; `dt` is always the time since the previous update.
- **Templates** take `std::floating_point T`; use `float` on microcontrollers with a
  single-precision FPU and `double` elsewhere.
- **Naming**: types in `PascalCase`, functions and variables in `snake_case`.

## Experimental API

`include/pidpp/experimental` and namespace `pidpp::experimental` contain source-unstable code.
They are not included by the facade. Since 0.2.0, the cascaded, gain-scheduled, and relay
headers there are aliases of the stable types and are kept only for source compatibility.
`lightweight_mpc.hpp` and `feedforward_controller.hpp` remain placeholders.

## Internal API

Headers under `include/pidpp/internal` are installed because the SDK is header-first, but they
are not part of the public API and may change without notice.

## Compatibility

PID++ follows semantic versioning. During `0.x`, minor versions may change the source API (all
changes are listed in the changelog). `1.0` will mark a stable public API. ABI stability is not
guaranteed before 1.0.
