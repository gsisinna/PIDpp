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
