#include "imu_1750.h"

using namespace boost::asio;

imu_1750::imu_1750(const std::string &portName) : serialPort(portName)
{
    init(portName);
    start(100);
}

void imu_1750::process()
{
    // read data

}

void imu_1750::setParam() const
{
    sp_->set_option(serial_port::baud_rate(115200));
    sp_->set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp_->set_option(serial_port::parity(serial_port::parity::none));
    sp_->set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp_->set_option(serial_port::character_size(8));
}
