#ifndef __VISUAL_H
#define __VISUAL_H

#include "phread.h"
#include <vector>
#include <list>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "imu.h"

class visualization:public thread
{
public:
    visualization();
    void set_pos(std::vector<Eigen::Vector3f> pos){pos_ = std::move(pos);}
    void set_quaternion(std::vector<Eigen::Quaternionf> q){q_ = std::move(q);}
    void set_imu(imu_bag data){imu_ = data;}
    void set_laser(std::vector<Eigen::Vector3f> laser,Eigen::Vector3f pos=Eigen::Vector3f(0,0,0))
    {
        laser_=laser;
        laser_pos_ = pos;
        pos_.emplace_back(pos);
    }
    ~visualization();
private:
    std::vector<Eigen::Vector3f> pos_;
    std::vector<Eigen::Quaternionf> q_;
    imu_bag imu_;
    std::vector<Eigen::Vector3f> laser_;
    Eigen::Vector3f laser_pos_;
    void process() override;
    void draw_trajectory();
    void draw_pose();
    void draw_imu();
    void draw_laser();

    static void* pthread_fun(void* __this);
};

#endif // __VISUAL_H
