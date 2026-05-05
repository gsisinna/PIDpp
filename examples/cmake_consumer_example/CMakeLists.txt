cmake_minimum_required(VERSION 3.20)

project(pidpp_consumer_example LANGUAGES CXX)

find_package(PIDPP CONFIG REQUIRED)

add_executable(pidpp_consumer main.cpp)
target_compile_features(pidpp_consumer PRIVATE cxx_std_20)
target_link_libraries(pidpp_consumer PRIVATE pidpp::pidpp)
