cmake_minimum_required(VERSION 3.5)
project(laser_slam)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(Pangolin 0.4 REQUIRED)

include_directories(
        ./common
        ./test
        ./driver
        /usr/include/eigen3
        ${Pangolin_INCLUDE_DIRS}
)

add_library(
        driver
        ./driver/laser.cpp
        ./driver/imu.cpp
        ./driver/serial_port.cpp
        ./driver/imu_1750.cpp
)
add_library(
        common
        ./common/visual.cpp
)
target_link_libraries(
        common
        ${Pangolin_LIBRARIES}
        -lpthread
        -lboost_system
)
################### test ########################

add_executable(test_laser
        ./test/test_laser.cpp
        )

target_link_libraries(
        test_laser
        driver
        common
)