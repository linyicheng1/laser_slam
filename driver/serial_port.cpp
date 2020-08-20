#include "serial_port.h"
#include <iostream>
#include <utility>

using namespace boost::asio;

/**
 * @brief 串口类的初始化
 * @param portName 串口的名称
 */
serialPort::serialPort(const std::string& portName)
{
    init(portName);
}
void serialPort::init(const std::string& portName)
{
    try
    {
        sp_ = new serial_port(ioSev_, portName);
        setParam();
    }
    catch (...)
    {
        std::cerr << "Exception Error: " << err_.message() << std::endl;
    }
}


/**
 * @brief 读取串口数据，做错误处理
 */
int serialPort::SP_Read(unsigned char *msg,int max_len)
{
    try
    {
        read(*sp_, boost::asio::buffer(msg, max_len), err_);
    }
    catch (...)
    {
        std::cerr << "Exception Error: " << err_.message() << std::endl;
    }
    return 0;
}

/**
 * @brief 写入串口数据，做错误处理
 */
int serialPort::SP_Write(unsigned char *msg, int len)
{
    try
    {
        write(*sp_, buffer(msg, (size_t)len), err_);
    }
    catch (...)
    {
        std::cerr << "Exception Error: " << err_.message() << std::endl;
    }
    return 0;
}