#ifndef MOONS_COMMS_HPP
#define MOONS_COMMS_HPP

#include <sstream>
// #include <libserial/SerialPort.h>
#include <iostream>
#include <string>
#include <cstdint>
#include <iomanip>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "diffdrive_ros2_control/base_comms.hpp"

// LibSerial::BaudRate convert_baud_rate(int baud_rate)
// {
//     // Just handle some common baud rates
//     switch (baud_rate)
//     {
//     case 1200:
//         return LibSerial::BaudRate::BAUD_1200;
//     case 1800:
//         return LibSerial::BaudRate::BAUD_1800;
//     case 2400:
//         return LibSerial::BaudRate::BAUD_2400;
//     case 4800:
//         return LibSerial::BaudRate::BAUD_4800;
//     case 9600:
//         return LibSerial::BaudRate::BAUD_9600;
//     case 19200:
//         return LibSerial::BaudRate::BAUD_19200;
//     case 38400:
//         return LibSerial::BaudRate::BAUD_38400;
//     case 57600:
//         return LibSerial::BaudRate::BAUD_57600;
//     case 115200:
//         return LibSerial::BaudRate::BAUD_115200;
//     case 230400:
//         return LibSerial::BaudRate::BAUD_230400;
//     default:
//         std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 115200" << std::endl;
//         return LibSerial::BaudRate::BAUD_115200;
//     }
// }

class MoonsComm : public BaseComm
{
public:

    MoonsComm()
    {
    }

    ~MoonsComm()
    {
    }





    bool ConnectComm(std::string port, int baud_rate, int timeout_ms)
    {

        // timeout_ms_ = timeout_ms;
        // try
        // {
        //     serial_.Open(port);
        //     serial_.SetBaudRate(convert_baud_rate(baud_rate));
        //     std::stringstream ss;
        //     ss << "0103000300020B34";
        //     std::cout<< "status Code"<<send_msg(ss.str());
            
        // }
        // catch (...)
        // {
        //     std::cout << "Bad Connection with serial port Error" << std::endl;
        // }

        // // just checking again
        // if (serial_.IsOpen())
        // {
        //     std::cout << "Serial is open" << std::endl;
        //     return true;
        // }
        // else
        // {
        //     std::cout << "Bad Connection with serial port Error" << std::endl;
        //     return false;
        // }

        // return false;
        return true;
    }

    bool DisconnectComm()
    {
        // serial_.Close();
        return true;
    }

    bool connected()
    {
        
        // std::stringstream ss;

        // ss<< "011008C9000204009600961738";//jog enable motor 1
        // send_msg( ss.str() );

        // ss<< "011008C800020400960096DBF9";//jog enable motor 2
        // send_msg( ss.str() );

        
        
        // return serial_.IsOpen();
    }


    void DriveCommand(double left_motor_rpm, double right_motor_rpm)
    {
        //send motor rpm 
        // rclcpp::spin(std::make_shared<RpmPublisher>());
       
        // try{
        //     std::string res=send_msg(vs.str());
        // }
        // catch(...){
        //     std::cout<< "Error in sending velocity : ";
        // }
    }

   

    bool ReadRPM(double &left_encoder_rpm, double &right_encoder_rpm)
    {
        // std::stringstream ss;
        // ss << "010307DD0004"<< calculateCRC16Modbus(ss.str());
        // std::string response = send_msg(ss.str());

        // int left_rpm_ = 0;
        // int right_rpm_ = 0;

        // if (!response.empty())
        // {
        //     std::cout<< response;
        //     // sscanf(response.c_str(), "S=%x:%x\r", &left_rpm_, &right_rpm_);

        //     // left_encoder_rpm = double(left_rpm_);
        //     // right_encoder_rpm = double(right_rpm_);
            
        //     // std::printf("motor encoders %.1f %.1f \n", right_encoder_rpm, left_encoder_rpm);

        //     return true;
        // }
        // else
        // {
        //     std::cout << "read timeout" << std::endl;
        //     return false;
        // }
        return true;
    }

private:
    
};

#endif // MOONS_COMMS_HPP