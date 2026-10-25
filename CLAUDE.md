# CLAUDE.md

Guidance for AI coding assistants working in this repository.

## Project

PID++ (`pidpp`) is a header-first C++20 library for deterministic PID control in robotics,
plus examples, tests, benchmarks, and an optional ImGui "Control Lab" tool. Public API lives in
`include/pidpp/`; the only compiled source is `src/version.cpp`.

## Commands

```sh
# Configure, build, test (unit tests + every example as a CTest scenario)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug -DPIDPP_ENABLE_SANITIZERS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure

# Single test / single example
./build/tests/pidpp_tests "dynamic clamping stops windup while proportional action saturates"
./build/examples/cascaded_servo_control --csv out.csv

# Optional targets
cmake -S . -B build -DPIDPP_BUILD_GUI=ON         # then: build/tools/gui/pidpp_control_lab [--headless]
cmake -S . -B build -DPIDPP_BUILD_BENCHMARKS=ON  # then: build/benchmarks/pidpp_update_benchmark
cmake -S . -B build -DPIDPP_BUILD_DOCS=ON && cmake --build build --target pidpp_docs

# Lint (CI uses these exact versions)
clang-format --dry-run --Werror <files>           # clang-format 17.0.6
clang-tidy -p build src/version.cpp               # clang-tidy 17, warnings are errors
```

## Architecture

- `core/` (Result/ErrorCode, numeric, time, saturation/deadband, angles) has no dependencies.
- `controllers/` builds on `core` and `filters`: `PidConfig` (aggregate + `validate()`),
  `PidController` (all logic in a private `step()`; `update()` and `update_with_feedforward()`
  are thin wrappers), `CascadedPidController`, `GainScheduledPidController`.
- `autotune/` relay experiment and tuning rules; `motion/` trapezoidal profile; `metrics/`
  offline analysis; `simulation/` plant models used by examples, tests, and the GUI.
- `experimental/` headers are aliases/placeholders, not included by `pidpp.hpp`.
- `tools/gui/lab_simulation.*` is GUI-free and runs headless in CI; `main.cpp` is the ImGui
  front-end.

## Invariants to preserve

- Controller `update()` paths: `noexcept`, no allocation, O(1), no logging or virtual calls.
  Invalid inputs return the previous output and set `state().last_dt_valid = false`.
- `validate()` never throws; it returns `Result` with a static message.
- The integrator stores accumulated error (error * s); `ki * integral` is the contribution.
  `set_gains`/`set_config` rescale it so the output does not jump.
- `include/pidpp/version.hpp` must match `project(VERSION)` in `CMakeLists.txt` (configure
  fails otherwise); tests read the expected version from CMake.
- Examples check their own acceptance criteria and return non-zero on failure. If you change
  library behavior, run the examples, not only the unit tests.
- Keep example noise deterministic (fixed-seed LCG), never `std::normal_distribution`, so
  results match across standard libraries.

## Conventions

- Types `PascalCase`, functions/variables `snake_case`, SI units, `dt` in seconds.
- Configuration structs are designed for designated initializers: append new fields at the end
  to keep existing initializers valid.
- Avoid namespace-scope names in examples that match SDK parameter names (`dt`, `target`, ...):
  MSVC warns (C4459).
- Line endings are LF (`.gitattributes`).
- Commit messages use Conventional Commits.
