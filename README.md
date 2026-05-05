# PID++ Control Engine

PID++ Control Engine is a C++20 SDK for deterministic PID control in robotics and embedded systems. The repository, headers, namespace, and CMake target use the ASCII-safe name `pidpp`.

## Who It Is For

PID++ is intended for robotics teams building motor drives, joint controllers, test rigs, embedded control firmware, and simulation-backed control workflows that need a small, predictable C++ API.

## Why It Exists

Many PID implementations are useful snippets but poor SDK foundations. PID++ focuses on:

- stable public API boundaries;
- explicit validation instead of exceptions in control paths;
- no dynamic allocation in `PidController<T>::update()`;
- deterministic controller execution;
- installable CMake packaging;
- room for commercial extensions such as cascaded PID, gain scheduling, adaptive control, feedforward, and lightweight MPC.

## Real-Time Safety Guarantee

For `pidpp::PidController<T>::update()`:

- no heap allocation;
- `noexcept`;
- deterministic O(1) complexity;
- no logging;
- no virtual dispatch;
- invalid `dt`, setpoint, or measurement preserves the previous output.

Validation, metrics, autotune, examples, and simulation helpers are intended to run outside hard real-time loops unless explicitly documented otherwise.

## Quick Start

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

## Installation

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPIDPP_BUILD_TESTS=OFF
cmake --build build
cmake --install build --prefix /opt/pidpp
```

## CMake Integration

```cmake
find_package(PIDPP CONFIG REQUIRED)
target_link_libraries(my_robot PRIVATE pidpp::pidpp)
```

Point `CMAKE_PREFIX_PATH` at the install prefix when needed:

```sh
cmake -S my_robot -B build -DCMAKE_PREFIX_PATH=/opt/pidpp
```

## Basic PID Example

```cpp
#include <pidpp/pidpp.hpp>

int main()
{
    pidpp::PidConfig<double> config{
        .kp = 1.2,
        .ki = 0.4,
        .kd = 0.05,
        .output_min = -12.0,
        .output_max = 12.0,
        .anti_windup = pidpp::AntiWindupStrategy::ConditionalIntegration,
        .derivative_mode = pidpp::DerivativeMode::OnMeasurement,
        .derivative_filter_tau = 0.02
    };

    auto result = config.validate();
    if (!result) {
        return 1;
    }

    pidpp::PidController<double> controller{config};
    const double command = controller.update(10.0, 8.5, 0.001);
    return command == 0.0 ? 1 : 0;
}
```

## Motor Velocity Example

See `examples/motor_speed_control.cpp` for a velocity loop around a lightweight DC motor model. It demonstrates derivative-on-measurement and conditional integration for saturated voltage commands.

## Build Options

- `PIDPP_BUILD_TESTS`: build tests.
- `PIDPP_BUILD_EXAMPLES`: build examples.
- `PIDPP_BUILD_BENCHMARKS`: build optional Google Benchmark targets.
- `PIDPP_BUILD_GUI`: build the optional PIDPP Control Lab GUI.
- `PIDPP_ENABLE_WARNINGS`: enable strict warnings for PIDPP targets.
- `PIDPP_ENABLE_SANITIZERS`: enable address/undefined sanitizers for PIDPP targets.
- `PIDPP_BUILD_DOCS`: reserved for documentation targets.
- `PIDPP_INSTALL`: install targets, headers, CMake config files, and pkg-config metadata.
- `PIDPP_EMBEDDED_PROFILE`: use embedded-friendly defaults that avoid examples, benchmarks, and docs.

The optional GUI requires native windowing/OpenGL development packages. On Ubuntu/Debian:

```sh
sudo apt-get install libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
```

## API Stability Policy

PID++ follows semantic versioning expectations:

- `0.x` may evolve as the SDK matures.
- `1.0` means the public API is considered stable.
- APIs under `pidpp::experimental` can change at any time.
- Headers under `include/pidpp/internal` are implementation details.
- C++ ABI stability is not promised before 1.0.

## Roadmap

- Cascaded PID controller composition.
- Gain scheduled PID using fixed-capacity or caller-owned storage.
- Adaptive control policies.
- Richer feedforward controllers.
- Lightweight MPC with bounded workspaces.
- Additional embedded profiles and package manager recipes.

## Non-Goals

- Heap allocation inside controller updates.
- Exception-driven real-time error handling.
- Virtual controller interfaces in hot paths.
- Heavy framework dependencies in the core SDK.
- ABI stability before 1.0.

## License

MIT. See `LICENSE`.
