cmake_minimum_required(VERSION 3.5)
project(laser_slam)
set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
find_package(Pangolin 0.4 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_conversions
        pcl_ros
        )

include_directories(
        ./common
        ./test
        ./driver
        /usr/include/eigen3
        ${Pangolin_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        /opt/ros/melodic/include/csm
)

add_library(
        driver
        ./driver/laser.cpp
        ./driver/imu.cpp
        ./driver/serial_port.cpp
        ./driver/imu_1750.cpp
        ./driver/odom.cpp
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
        ${catkin_LIBRARIES}
)
target_link_libraries(
        driver
        ${catkin_LIBRARIES}
        /opt/ros/melodic/lib/libcsm.so
)
################### test ########################

add_executable(test_laser
        ./test/test_laser.cpp
        )

target_link_libraries(
        test_laser
        driver
        common
        ${catkin_LIBRARIES}
        ${Pangolin_LIBRARIES}
        /opt/ros/melodic/lib/libcsm.so
)

#add_executable(test_imu
#        ./test/test_imu.cpp
#        )
#
#target_link_libraries(
#        test_imu
#        driver
#        common
#)
