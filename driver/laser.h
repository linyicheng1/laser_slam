#ifndef __LASER_H
#define __LASER_H
#include "../common/phread.h"
#include "../common/common.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <csm/csm_all.h>
#include "eigen3/Eigen/Core"

class laser
{
public:
    struct laser_config
    {
        float max_distance_;
        float min_distance_;
        float error_angle_;
        bool verbose_;
        laser_config(float max_distance,
        float min_distance,
        float error_angle,
        bool verbose)
        {
            max_distance_ = max_distance;
            min_distance_ = min_distance;
            error_angle_ = error_angle;
            verbose_ = verbose;
        }
    }config_{20,0.1,0.01,true};
    laser() = default;
    ~laser() =default;
    virtual std::vector<Eigen::Vector3f> get_frame() = 0;

private:
    void distortion();
};

class laser_ROS: public laser,thread
{
public:
    laser_ROS();
    ~laser_ROS() = default;
    std::vector<Eigen::Vector3f> get_frame() override;
    Eigen::Vector3f get_frame_pos()
    {
        mutex_.lock();
        Eigen::Vector3f tmp = scan_pos_cal;
        mutex_.unlock();
        return tmp;
    }
    std::vector<Eigen::Vector3f> get_pl_icp()
    {
        mutex_.lock();
        std::vector<Eigen::Vector3f> tmp(pose_.size());
        for(int i=0;i<pose_.size();i++)
        {
            for(int j=0;j<3;j++)
            {
                tmp[i][j] = pose_[i][j];
            }
        }
        mutex_.unlock();
        return tmp;
    }

private:
    void process() override;
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr &_laserScanMsg);
    void LaserScanToLDP(sensor_msgs::LaserScan *pScan,LDP& ldp);
    Eigen::Vector3f  PIICPBetweenTwoFrames(LDP& currentLDPScan,Eigen::Vector3f tmprPose);
    void SetPIICPParams();
    //进行PI-ICP需要的变量
    LDP m_prevLDP;
    LDP m_currentLDP;
    sm_params m_PIICPParams;
    sm_result m_OutputResult;
    std::vector<Eigen::Vector3f> pose_;
    Eigen::Vector3f scan_pos_cal;
};

#endif //__LASER_H
