#include "laser.h"
#include "eigen3/Eigen/Core"

void laser::distortion()
{

}

std::vector<Eigen::Vector3f> laser_ROS::get_frame()
{
    mutex_.lock();
    if(m_currentLDP != NULL)
    {
        std::vector<Eigen::Vector3f> data(m_currentLDP->nrays);
        for(int i=0;i<m_currentLDP->nrays;i++)
        {
            data.at(i)[0] = m_currentLDP->valid[i];
            data.at(i)[1] = m_currentLDP->readings[i];
            data.at(i)[2] = m_currentLDP->theta[i];
        }
        mutex_.unlock();
        return data;
    }
    mutex_.unlock();
    return std::vector<Eigen::Vector3f>{};
}

laser_ROS::laser_ROS()
{
    start(1000000);
    scan_pos_cal.setZero();
}

void laser_ROS::process()
{
    int argc=0;char **argv = nullptr;
    ros::init(argc, argv, "laser_test");
    ros::NodeHandle private_nh_("~");
    SetPIICPParams();
    ros::Subscriber sub = private_nh_.subscribe("/sick_scan", 100,&laser_ROS::laser_callback,this);
    ros::spin();
}

void laser_ROS::laser_callback(const sensor_msgs::LaserScan_<std::allocator<void>>::ConstPtr &_laserScanMsg)
{
    sensor_msgs::LaserScan scan;
    scan = *_laserScanMsg;
    //把当前的激光数据转换为 pl-icp能识别的数据 & 进行矫正
    //d_point_scan就是用激光计算得到的两帧数据之间的旋转 & 平移

    Eigen::Vector3f d_point_scan;
    static Eigen::Vector3f last_scan;
    if(m_prevLDP != NULL)
    {
        mutex_.lock();
        LaserScanToLDP(&scan,m_currentLDP);
        d_point_scan = PIICPBetweenTwoFrames(m_currentLDP,last_scan);

        Eigen::Matrix3f trans;
        float c = cosf(scan_pos_cal(2));
        float s = sinf(scan_pos_cal(2));
        trans<<c,-s,0,
                s, c,0,
                0, 0,1;
        scan_pos_cal+=(trans*d_point_scan);
        pose_.emplace_back(scan_pos_cal);
        mutex_.unlock();

        last_scan = d_point_scan;
    }
    else
    {
        mutex_.lock();
        pose_.emplace_back(Eigen::Vector3f(0,0,0));
        mutex_.unlock();
        LaserScanToLDP(&scan,m_prevLDP);
    }
}

void laser_ROS::LaserScanToLDP(sensor_msgs::LaserScan *pScan, LDP &ldp)
{
    int nPts = pScan->intensities.size();
    ldp = ld_alloc_new(nPts);

    for(int i = 0;i < nPts;i++)
    {
        double dist = pScan->ranges[i];
        if(dist > config_.min_distance_ && dist < config_.max_distance_)
        {
            ldp->valid[i] = 1;
            ldp->readings[i] = dist;
        }
        else
        {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1;
        }
        ldp->theta[i] = pScan->angle_min+pScan->angle_increment*i;
    }
    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[nPts-1];

    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}

Eigen::Vector3f laser_ROS::PIICPBetweenTwoFrames(LDP &currentLDPScan, Eigen::Vector3f tmprPose)
{
    m_prevLDP->odometry[0] = 0.0;
    m_prevLDP->odometry[1] = 0.0;
    m_prevLDP->odometry[2] = 0.0;

    m_prevLDP->estimate[0] = 0.0;
    m_prevLDP->estimate[1] = 0.0;
    m_prevLDP->estimate[2] = 0.0;

    m_prevLDP->true_pose[0] = 0.0;
    m_prevLDP->true_pose[1] = 0.0;
    m_prevLDP->true_pose[2] = 0.0;

    //设置匹配的参数值
    m_PIICPParams.laser_ref = m_prevLDP;
    m_PIICPParams.laser_sens = currentLDPScan;

    m_PIICPParams.first_guess[0] = tmprPose(0);
    m_PIICPParams.first_guess[1] = tmprPose(1);
    m_PIICPParams.first_guess[2] = tmprPose(2);

    m_OutputResult.cov_x_m = 0;
    m_OutputResult.dx_dy1_m = 0;
    m_OutputResult.dx_dy2_m = 0;

    sm_icp(&m_PIICPParams,&m_OutputResult);

    //nowPose在lastPose中的坐标
    Eigen::Vector3f  rPose;
    if(m_OutputResult.valid)
    {
        //得到两帧激光之间的相对位姿
        rPose(0)=(m_OutputResult.x[0]);
        rPose(1)=(m_OutputResult.x[1]);
        rPose(2)=(m_OutputResult.x[2]);
        if(config_.verbose_)
        {
            std::cout <<"Iter:"<<m_OutputResult.iterations<<std::endl;
            std::cout <<"Corr:"<<m_OutputResult.nvalid<<std::endl;
            std::cout <<"Erro:"<<m_OutputResult.error<<std::endl;
            std::cout <<"PI ICP GOOD"<<std::endl;
        }
    }
    else
    {
        std::cout <<"PI ICP Failed!!!!!!!"<<std::endl;
        rPose = tmprPose;
    }
    m_prevLDP = currentLDPScan;
    return rPose;
}

void laser_ROS::SetPIICPParams()
{
    //设置激光的范围
    m_PIICPParams.min_reading = config_.min_distance_;
    m_PIICPParams.max_reading = config_.max_distance_;

    //设置位姿最大的变化范围
    m_PIICPParams.max_angular_correction_deg = 20.0;
    m_PIICPParams.max_linear_correction = 1;

    //设置迭代停止的条件
    m_PIICPParams.max_iterations = 30;
    m_PIICPParams.epsilon_xy = 0.000001;
    m_PIICPParams.epsilon_theta = 0.0000001;

    //设置correspondence相关参数
    m_PIICPParams.max_correspondence_dist = 1;
    m_PIICPParams.sigma = 0.01;
    m_PIICPParams.use_corr_tricks = 1;

    //设置restart过程，因为不需要restart所以可以不管
    m_PIICPParams.restart = 0;
    m_PIICPParams.restart_threshold_mean_error = 0.01;
    m_PIICPParams.restart_dt = 1.0;
    m_PIICPParams.restart_dtheta = 0.1;

    //设置聚类参数
    m_PIICPParams.clustering_threshold = 0.2;

    //用最近的10个点来估计方向
    m_PIICPParams.orientation_neighbourhood = 10;

    //设置使用PI-ICP
    m_PIICPParams.use_point_to_line_distance = 1;

    //不进行alpha_test
    m_PIICPParams.do_alpha_test = 0;
    m_PIICPParams.do_alpha_test_thresholdDeg = 5;

    //设置trimmed参数 用来进行outlier remove
    m_PIICPParams.outliers_maxPerc = 0.9;
    m_PIICPParams.outliers_adaptive_order = 0.7;
    m_PIICPParams.outliers_adaptive_mult = 2.0;

    //进行visibility_test 和 remove double
    m_PIICPParams.do_visibility_test = 1;
    m_PIICPParams.outliers_remove_doubles = 1;
    m_PIICPParams.do_compute_covariance = 0;
    m_PIICPParams.debug_verify_tricks = 0;
    m_PIICPParams.use_ml_weights = 0;
    m_PIICPParams.use_sigma_weights = 0;
}


