#ifndef BASE_COMMS_HPP
#define BASE_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <string>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
    // Just handle some common baud rates
    switch (baud_rate)
    {
    case 1200:
        return LibSerial::BaudRate::BAUD_1200;
    case 1800:
        return LibSerial::BaudRate::BAUD_1800;
    case 2400:
        return LibSerial::BaudRate::BAUD_2400;
    case 4800:
        return LibSerial::BaudRate::BAUD_4800;
    case 9600:
        return LibSerial::BaudRate::BAUD_9600;
    case 19200:
        return LibSerial::BaudRate::BAUD_19200;
    case 38400:
        return LibSerial::BaudRate::BAUD_38400;
    case 57600:
        return LibSerial::BaudRate::BAUD_57600;
    case 115200:
        return LibSerial::BaudRate::BAUD_115200;
    case 230400:
        return LibSerial::BaudRate::BAUD_230400;
    default:
        std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
        return LibSerial::BaudRate::BAUD_57600;
    }
}

class BaseComm
{
public:

    virtual bool ConnectComm(std::string port, int baud_rate, int timeout_ms) = 0;

    virtual bool DisconnectComm() = 0;

    virtual bool connected() = 0;

    virtual void DriveCommand(double left_motor_rpm, double right_motor_rpm) = 0;

    virtual bool ReadRPM(double &left_encoder_rpm, double &right_encoder_rpm) = 0;

};

#endif // BASE_COMMS_HPP