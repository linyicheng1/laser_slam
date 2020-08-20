#include <iostream>
#include "laser.h"

int main()
{
    laser_ROS test;
    test.get_frame();

    std::cout<<"Hello slam!"<<std::endl;
}