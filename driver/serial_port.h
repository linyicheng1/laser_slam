#ifndef __SERIAL_PORT_H
#define __SERIAL_PORT_H

#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

class serialPort
{
public:
    serialPort() = default;
    explicit serialPort(const std::string& portName);
    void init(const std::string& portName);
    virtual void setParam() const = 0;
    ~serialPort() = default;
    int SP_Write(unsigned char *msg, int len);
    int SP_Read(unsigned char *msg,int max_len);
    boost::asio::serial_port *sp_{};
    boost::asio::io_service ioSev_;

protected:

    std::string portName_;
    boost::system::error_code err_;
};

#endif // __SERIAL_PORT