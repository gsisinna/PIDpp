# Examples

The examples are small SDK integration references.

## `basic_pid.cpp`

Shows configuration validation and a single PID update.

## `motor_speed_control.cpp`

Controls a simulated DC motor velocity with voltage saturation.

## `joint_position_control.cpp`

Controls a second-order joint position with torque saturation.

## `step_response_analysis.cpp`

Runs offline step-response metrics on a first-order plant.

## `cmake_consumer_example`

Demonstrates downstream use with:

```cmake
find_package(PIDPP CONFIG REQUIRED)
target_link_libraries(pidpp_consumer PRIVATE pidpp::pidpp)
```

## PIDPP Control Lab

The optional GUI tool is intended for quick controller experiments and result visualization:

```sh
cmake -S . -B build-gui -DPIDPP_BUILD_GUI=ON -DPIDPP_BUILD_TESTS=OFF
cmake --build build-gui --target pidpp_control_lab
./build-gui/tools/gui/pidpp_control_lab
```

It uses Dear ImGui and ImPlot through CMake `FetchContent`, so GUI dependencies are not required for normal SDK consumers.

On Ubuntu/Debian, install native windowing dependencies before enabling it:

```sh
sudo apt-get install libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
```
