add_executable(basic_pid basic_pid.cpp)
target_link_libraries(basic_pid PRIVATE pidpp::pidpp)

add_executable(motor_speed_control motor_speed_control.cpp)
target_link_libraries(motor_speed_control PRIVATE pidpp::pidpp)

add_executable(joint_position_control joint_position_control.cpp)
target_link_libraries(joint_position_control PRIVATE pidpp::pidpp)

add_executable(step_response_analysis step_response_analysis.cpp)
target_link_libraries(step_response_analysis PRIVATE pidpp::pidpp)

if(PIDPP_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    foreach(example_target IN ITEMS
        basic_pid
        motor_speed_control
        joint_position_control
        step_response_analysis
    )
        target_compile_options(${example_target} PRIVATE ${PIDPP_SANITIZER_COMPILE_OPTIONS})
        target_link_options(${example_target} PRIVATE ${PIDPP_SANITIZER_LINK_OPTIONS})
    endforeach()
endif()
