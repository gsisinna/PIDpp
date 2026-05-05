# CMake Integration

PID++ exports a modern CMake package.

## Install

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPIDPP_BUILD_TESTS=OFF
cmake --build build
cmake --install build --prefix /opt/pidpp
```

## Consume

```cmake
find_package(PIDPP CONFIG REQUIRED)
target_link_libraries(my_robot PRIVATE pidpp::pidpp)
```

Configure your downstream project with:

```sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/opt/pidpp
```

The exported target carries include directories and C++20 requirements. PID++ warning and sanitizer options are private to PID++ targets and are not forced onto downstream consumers.

## Optional GUI Tool

`PIDPP_BUILD_GUI=ON` builds `pidpp_control_lab`, a native Dear ImGui and ImPlot tool for tuning and visualization. It is off by default and is not installed as part of the SDK package.

On Linux, the GUI target uses GLFW with X11 by default. Install the X11/OpenGL development packages required by GLFW before configuring the GUI build.

## Package Metadata

The install step generates:

- `PIDPPConfig.cmake`
- `PIDPPConfigVersion.cmake`
- `PIDPPTargets.cmake`
- `pidpp.pc`
