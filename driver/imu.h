#ifndef __IMU_H
#define __IMU_H
#include "serial_port.h"
#include "phread.h"
#include <list>

typedef struct bag
{
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float angle_x;
    float angle_y;
    float angle_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float time_stamp;
}imu_bag;

typedef std::list<imu_bag> imu_data;
class imu
{
public:
    imu() =default;
    ~imu() = default;
    imu_data get_imu_data(){return data_;}
    imu_data data_;
    bool verbose = true;
};

#endif //__IMU_H
