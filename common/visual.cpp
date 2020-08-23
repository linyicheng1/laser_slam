#include "visual.h"
#include <pangolin/pangolin.h>

visualization::visualization(/* args */)
{
    delay_ = 100000;
    pthread_create(&thread_, nullptr, pthread_fun, (void *) this);
}

void visualization::draw_trajectory()
{
    glLineWidth(3);
    if(pos_.size()<2)
        return;
    for (size_t i = 1; i < pos_.size()-1; i++)
    {
        glColor3f(0.0, 1.0, 0.0);
        glBegin(GL_LINES);
        auto p1 = pos_[i-1], p2 = pos_[i];
        glVertex3f(p1[0], p1[1], 0);
        glVertex3f(p2[0], p2[1], 0);
        glEnd();
    }
}

void visualization::draw_pose()
{
    // not all
    for(size_t i=0;i<q_.size();i+=20)
    {
        // 画每个位姿的三个坐标轴
        Eigen::Vector3f Ow = pos_[i];
        q_[i].normalized();
        Eigen::Vector3f Xw = q_[i].toRotationMatrix() * (0.1f * Eigen::Vector3f(1, 0, 0)) + Ow;
        Eigen::Vector3f Yw = q_[i].toRotationMatrix() * (0.1f * Eigen::Vector3f(0, 1, 0)) + Ow;
        Eigen::Vector3f Zw = q_[i].toRotationMatrix() * (0.1f * Eigen::Vector3f(0, 0, 1)) + Ow;

        glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(Ow[0], Ow[1], Ow[2]);
        glVertex3f(Xw[0], Xw[1], Xw[2]);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(Ow[0], Ow[1], Ow[2]);
        glVertex3f(Yw[0], Yw[1], Yw[2]);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(Ow[0], Ow[1], Ow[2]);
        glVertex3f(Zw[0], Zw[1], Zw[2]);
        glEnd();
    }
}

void visualization::draw_imu()
{
    Eigen::Vector3f Ow;

    Eigen::Matrix3f rotation_matrix;
    rotation_matrix  = Eigen::AngleAxisf(imu_.angle_z, Eigen::Vector3f::UnitZ()) *
                       Eigen::AngleAxisf(imu_.angle_y, Eigen::Vector3f::UnitY()) *
                       Eigen::AngleAxisf(imu_.angle_x, Eigen::Vector3f::UnitX());

    Eigen::Vector3f Xw = rotation_matrix * (imu_.acc_x * Eigen::Vector3f(1, 0, 0)) + Ow;
    Eigen::Vector3f Yw = rotation_matrix * (imu_.acc_y * Eigen::Vector3f(0, 1, 0)) + Ow;
    Eigen::Vector3f Zw = rotation_matrix * (imu_.acc_z * Eigen::Vector3f(0, 0, 1)) + Ow;

    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(Ow[0], Ow[1], Ow[2]);
    glVertex3f(Xw[0], Xw[1], Xw[2]);
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f(Ow[0], Ow[1], Ow[2]);
    glVertex3f(Yw[0], Yw[1], Yw[2]);
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(Ow[0], Ow[1], Ow[2]);
    glVertex3f(Zw[0], Zw[1], Zw[2]);
    glEnd();
}

void visualization::draw_laser()
{

    glPointSize(3);
    for(auto pt:laser_)
    {
        if(pt[0]>0)
        {
            glColor3f(0.0, 0.0, 1.0);
            glBegin(GL_POINTS);
            glVertex3f(pt[1]*cos(pt[2]+laser_pos_[2]) + laser_pos_[0], pt[1]*sin(pt[2]+laser_pos_[2]) + laser_pos_[1], 0);
            glEnd();
        }
    }
}

void visualization::process()
{
    draw_trajectory();
    draw_pose();
    //draw_imu();
    draw_laser();
}

visualization::~visualization()
= default;

void* visualization::pthread_fun(void* __this)
{
    auto * _this =(thread *)__this;
    pangolin::CreateWindowAndBind("Main",1080,960);
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1080,960,420,420,640,480,0.2,100),
            pangolin::ModelViewLookAt(1,1,1, 0,0,0, pangolin::AxisZ)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);
    while (true)
    {
        try
        {
            // Clear screen and activate view to render into
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glClearColor(1.0f,1.0f,1.0f,1.0f);
            d_cam.Activate(s_cam);

            // draw 3d view
            _this->process();

            pangolin::FinishFrame();
        }
        catch (...)
        {
            break;
        }
        usleep(_this->delay_);
    }
}
