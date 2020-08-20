#ifndef __IMU_1750_H
#define __IMU_1750_H
#include "imu.h"
#include "serial_port.h"
#include "phread.h"
#include <string>

class imu_1750: public imu,serialPort,thread
{
public:
    imu_1750() = default;
    explicit imu_1750(const std::string& portName);
    ~imu_1750() = default;

private:
    void process() override;
    void setParam() const override;
};

#endif //__IMU_1750_H
