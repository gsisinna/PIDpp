set(PIDPP_EXAMPLES
    basic_pid
    step_response_analysis
    motor_speed_control
    joint_position_control
    cascaded_servo_control
    trajectory_tracking
    arm_gravity_compensation
    gain_scheduling_payload
    mobile_robot_navigation
    drone_altitude_hold
    relay_autotune
    sensor_noise_filtering
)

foreach(example_target IN LISTS PIDPP_EXAMPLES)
    add_executable(${example_target} ${example_target}.cpp)
    target_link_libraries(${example_target} PRIVATE pidpp::pidpp)
    target_include_directories(${example_target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

    if(PIDPP_ENABLE_WARNINGS)
        if(MSVC)
            target_compile_options(${example_target} PRIVATE /W4 /permissive-)
        else()
            target_compile_options(${example_target} PRIVATE -Wall -Wextra -Wpedantic)
        endif()
    endif()

    if(PIDPP_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(${example_target} PRIVATE ${PIDPP_SANITIZER_COMPILE_OPTIONS})
        target_link_options(${example_target} PRIVATE ${PIDPP_SANITIZER_LINK_OPTIONS})
    endif()

    # Every example checks its own acceptance criteria and exits non-zero on
    # failure, so running them doubles as an end-to-end regression suite.
    if(PIDPP_BUILD_TESTS)
        add_test(NAME example.${example_target} COMMAND ${example_target})
    endif()
endforeach()
