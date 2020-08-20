#ifndef __LASER_H
#define __LASER_H
#include "phread.h"
#include "common.h"

class laser
{
public:
    struct laser_config
    {
        float max_distance_;
        float min_distance_;
        float error_angle_;
        bool verbose;
    }config_{};
    laser() = default;
    ~laser() =default;
    virtual void get_frame() = 0;

private:
    void distortion();
};

class laser_ROS: public laser
{
public:
    laser_ROS() = default;
    ~laser_ROS() = default;
    void get_frame() override;
};

#endif //__LASER_H
