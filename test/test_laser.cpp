#include <iostream>
#include "../driver/laser.h"
#include "../common/visual.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{

    laser_ROS test;
    visualization vi;
    //test.get_frame();
    while (true)
    {
        vi.set_laser(test.get_frame(),test.get_frame_pos());
        usleep(100000);
    }
}