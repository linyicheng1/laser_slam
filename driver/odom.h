#ifndef __ODOM_H
#define __ODOM_H
#include <eigen3/Eigen/Core>
#include "phread.h"
#include <vector>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>

class odom
{
public:
    odom() = default;
    ~odom() = default;
    virtual void get_odom(std::vector<Eigen::Vector3f> &t,std::vector<Eigen::Quaternionf> &q)
    {
        q = q_;
        t = t_;
    }
    std::vector<Eigen::Vector3f> t_;
    std::vector<Eigen::Quaternionf> q_;
};
class odom_ros: public odom,thread
{
public:
    odom_ros();
    ~odom_ros() = default;

private:
    void process() override;
};

#endif //__ODOM_H
