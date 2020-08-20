cmake_minimum_required(VERSION 3.5)
project(laser_slam)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)

include_directories(
        ./common
        ./test
        ./driver
        /usr/include/eigen3
)

add_library(
        driver
        ./driver/laser.cpp
)
################### test ########################

add_executable(test_laser
        ./test/test_laser.cpp
        )

target_link_libraries(
        test_laser
        driver
)