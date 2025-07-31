#ifndef ROBOTEQ_COMMS_HPP
#define ROBOTEQ_COMMS_HPP

#include "diffdrive_ros2_control/base_comms.hpp"

class RoboteqComm : public BaseComm
{
public:
    RoboteqComm() {}

    ~RoboteqComm() {}

    bool ConnectComm(std::string port, int baud_rate, int timeout_ms)
    {

        timeout_ms_ = timeout_ms;
        try
        {
            serial_.Open(port);
            serial_.SetBaudRate(convert_baud_rate(baud_rate));

            // turning the echo off
            std::stringstream ss;
            ss << "^ECHOF 1\r";
            send_msg(ss.str());
        }
        catch (...)
        {
            std::cout << "Bad Connection with serial port Error" << std::endl;
        }

        // just checking again
        if (serial_.IsOpen())
        {
            std::cout << "Serial is open" << std::endl;
            return true;
        }
        else
        {
            std::cout << "Bad Connection with serial port Error" << std::endl;
            return false;
        }

        return false;
    }

    bool DisconnectComm()
    {
        serial_.Close();
        return true;
    }

    bool connected()
    {
        return serial_.IsOpen();
    }

    std::string send_msg(const std::string &msg_to_send, bool print_output = false)
    {
        serial_.FlushIOBuffers(); // Just in case
        serial_.Write(msg_to_send);
        serial_.DrainWriteBuffer(); // Wait until the data has actually been transmitted.

        std::string response = "";
        try
        {
            // Responses end with \r so we will read up to (and including) the \r.
            serial_.ReadLine(response, '\r', timeout_ms_);
        }
        catch (const LibSerial::ReadTimeout &)
        {
            std::cerr << "The ReadLine() call has timed out." << std::endl;
        }

        if (print_output)
        {
            std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
        }

        return response;
    }

    void DriveCommand(double left_motor_rpm, double right_motor_rpm)
    {
        std::stringstream ls, rs;
        ls << "!S " << 1 << " " << left_motor_rpm << "\r";
        send_msg(ls.str());
        rs << "!S " << 2 << " " << right_motor_rpm << "\r";
        send_msg(rs.str());
    }

    bool ReadEncoderValue(double &left_encoder_feedback, double &right_encoder_feedback)
    {
        std::stringstream ss;
        ss << "?CB\r";
        std::string response = send_msg(ss.str());

        int left_rpm_ = 0;
        int right_rpm_ = 0;

        if (!response.empty())
        {
            sscanf(response.c_str(), "CB=%d:%d\r", &right_rpm_, &left_rpm_);
            left_encoder_feedback = double(left_rpm_);
            right_encoder_feedback = double(right_rpm_);
            std::printf("HALL DUAL %f %f \n", left_encoder_feedback, right_encoder_feedback);
            return true;
        }
        else
        {
            std::cout << "read timeout" << std::endl;
            return false;
        }
    }

    bool ReadRPM(double &left_encoder_rpm, double &right_encoder_rpm)
    {
        std::stringstream ss;
        ss << "?S\r"; //?S for encoder calc RPM and ?BS for hall effect sensor in brushless motor calc RPM
        std::string response = send_msg(ss.str());

        int left_rpm_ = 0;
        int right_rpm_ = 0;

        if (!response.empty())
        {
            sscanf(response.c_str(), "S=%d:%d\r", &right_rpm_, &left_rpm_);

            left_encoder_rpm = double(left_rpm_);
            right_encoder_rpm = double(right_rpm_);

            std::printf("motor encoders %.1f %.1f \n", right_encoder_rpm, left_encoder_rpm);

            return true;
        }
        else
        {
            std::cout << "read timeout" << std::endl;
            return false;
        }
    }

private:
    LibSerial::SerialPort serial_;
    int timeout_ms_;
};

#endif // ROBOTEQ_COMMS_HPP