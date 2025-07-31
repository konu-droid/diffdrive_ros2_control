#ifndef ZLAC8015_COMMS_HPP
#define ZLAC8015_COMMS_HPP

#include "diffdrive_ros2_control/base_comms.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <stdint.h>
#include <vector>
#include <cstdio>
#include <cstring>
#include <chrono>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

/*================= CRC Check ==============*/

constexpr unsigned short crc16_table[] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

static inline unsigned short crc16(const unsigned char *data, unsigned short len)
{

    unsigned char i;

    unsigned int crc = 0xFFFF;

    while (len--)
    {

        i = (unsigned char)(crc ^ *data++);

        crc = (crc >> 8) ^ crc16_table[i];
    }

    return (unsigned short)crc;
};

/*================= CRC Check End ==============*/

class ZLAC8015
{
protected:
    std::chrono::time_point<std::chrono::steady_clock> start, end;

    uint8_t hex_cmd[8] = {0};
    uint8_t receive_hex[15] = {0};
    uint8_t ID = 0x00;
    const uint8_t READ = 0x03;
    const uint8_t WRITE = 0x06;
    const uint8_t CONTROL_REG[2] = {0X20, 0X31};
    const uint8_t ENABLE[2] = {0x00, 0X08};
    const uint8_t DISABLE[2] = {0x00, 0X07};
    const uint8_t OPERATING_MODE[2] = {0X20, 0X32};
    const uint8_t VEL_MODE[2] = {0x00, 0X03};
    const uint8_t SET_RPM[2] = {0x20, 0X3A};
    const uint8_t GET_RPM[2] = {0x20, 0X2C};
    const uint8_t SET_ACC_TIME[2] = {0x20, 0X37};
    const uint8_t SET_DECC_TIME[2] = {0x20, 0X38};
    const uint8_t SET_VEL_SMOOTHING[2] = {0x20, 0X18};
    const uint8_t SET_FORWARD_OUTPUT_SMOOTHING[2] = {0x20, 0X1B};
    const uint8_t SET_KP[2] = {0x20, 0X1D};
    const uint8_t SET_KI[2] = {0x20, 0X1E};
    const uint8_t SET_KF[2] = {0x20, 0X1F};
    const uint8_t INITIAL_SPEED[2] = {0X20, 0X08};
    const uint8_t MAX_SPEED[2] = {0X20, 0X0A};
    const uint8_t ACTUAL_POSITION_H[2] = {0X20, 0X2A};
    const uint8_t ACTUAL_POSITION_L[2] = {0X20, 0X2B};

    // HELPER Functions *************************************************************

    void sleep(unsigned long milliseconds)
    {
#ifdef _WIN32
        Sleep(milliseconds); // 100 ms
#else
        usleep(milliseconds * 1000); // 100 ms
#endif
    }

    /**
     * @brief calculates the crc and stores it in the hex_cmd array, so there is no return value
     */
    void calculate_crc()
    {
        // calculate crc and append to hex cmd
        unsigned short result = crc16(hex_cmd, sizeof(hex_cmd) - 2);
        hex_cmd[6] = result & 0xFF;
        hex_cmd[7] = (result >> 8) & 0xFF;
    }

    /**
     * @brief reads from the serial port and saves the string into the receive_hex array
     * @param num_bytes how many bytes to read from the buffer
     * @return return 0 when OK, 1 if crc error
     */
    uint8_t read_hex(uint8_t num_bytes)
    {
        std::string read_string;

        try
        {
            serial_.Read(read_string, num_bytes, timeout_ms_);
        }
        catch (const LibSerial::ReadTimeout &)
        {
            std::cerr << "The serial read call has timed out." << std::endl;
        }

        // convert string to hex
        for (uint8_t i = 0; i < uint8_t(read_string.size()); i++)
        {
            receive_hex[i] = uint8_t(read_string[i]);
            // printf("rec %d, %02x\n", i, receive_hex[i]);
        }

        // crc check of received data
        if (crc16(receive_hex, num_bytes) != 0)
        {
            // printf("crc checking error\n");
            return 1;
        }
        return 0;
    }

    /**
     * @brief print the hex command for debugging
     */
    void print_hex_cmd() const
    {
        // print
        for (int i = 0; i < 8; i++)
        {
            printf("%d, %02x\n", i, hex_cmd[i]);
        }
    }

    /**
     * @brief print received hex for debugging
     */
    void print_rec_hex() const
    {
        // print
        for (int i = 0; i < 8; i++)
        {
            printf("rec: %d, %02x\n", i, receive_hex[i]);
        }
    }

public:
    /**
     * @brief open serial port communication
     * @param port COM port eg. "/dev/ttyUSB0"
     * @param baudRate default baudrate is 115200
     * @param _ID Set the modbus ID of the motor driver in HEX, default 0x00
     */
    bool begin(std::string port, int baudrate, uint8_t ID)
    {
        this->ID = ID;
        try
        {
            serial_.Open(port);
            serial_.SetBaudRate(convert_baud_rate(baudrate));
            serial_.FlushIOBuffers();
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
        // std::cout << "SERIAL OK!" << std::endl;
    }

    /**
     * @param rpm
     * @return alwasy 0
     */
    uint8_t set_rpm(int16_t rpm)
    {
        // memset(hex_cmd, 0, sizeof(hex_cmd));
        hex_cmd[0] = ID;
        hex_cmd[1] = WRITE;
        hex_cmd[2] = SET_RPM[0];
        hex_cmd[3] = SET_RPM[1];

        hex_cmd[4] = (rpm >> 8) & 0xFF;
        hex_cmd[5] = rpm & 0xFF;

        calculate_crc();

        for (int i = 0; i < int(sizeof(hex_cmd) / sizeof(uint8_t)); i++)
        {
            serial_.WriteByte(hex_cmd[i]);
        }
        serial_.DrainWriteBuffer();
        read_hex(8);

        return 0;
    }

    /**
     * @return rpm measured from wheel
     */
    float get_rpm()
    {
        // // memset(hex_cmd, 0, sizeof(hex_cmd));
        // hex_cmd[0] = ID;
        // hex_cmd[1] = READ;
        // hex_cmd[2] = GET_RPM[0];
        // hex_cmd[3] = GET_RPM[1];

        // hex_cmd[4] = 0x00;
        // hex_cmd[5] = 0x01;

        // calculate_crc();
        // // print_hex_cmd();

        // // TODO isn't save, to continue reading infinite in case of error
        // // do // repeat sending and read command as long as it has crc error
        // // {
        // //     serial_.write(hex_cmd, 8);
        // //     // print_rec_hex();
        // // } while (read_hex(7) );

        // serial_.write(hex_cmd, 8);
        // read_hex(7);
        int16_t rpm_tenth = receive_hex[8] + (receive_hex[7] << 8);
        return (float)rpm_tenth / 10.0f;
    }

    /**
     * @return Actual torque feedback, unit: A
     */
    float get_torque()
    {
        int16_t torque = receive_hex[10] + (receive_hex[9] << 8);
        return (float)torque / 10.0f;
    }

    /**
     * @return Error feedback,
     *          0000h: no error;
     *          0001h: over-voltage;
     *          0002h: under-voltage;
     *          0004h: over-current;
     *          0008h: overload;
     *          0010h: current is out of tolerance;
     *          0020h: encoder is out of tolerance;
     *          0040h: speed is out of tolerance;
     *          0080h: reference voltage error;
     *          0100h: EEPROM read and write error;
     *          0200h: Hall error;
     *          0400h: motor temperature is too high.
     */
    uint16_t get_error()
    {
        return receive_hex[12] + (receive_hex[11] << 8);
    }

    /**
     * @return Actual position feedback, unit: counts
     */
    int32_t get_position()
    {
        // // memset(hex_cmd, 0, sizeof(hex_cmd));
        // hex_cmd[0] = ID;
        // hex_cmd[1] = READ;
        // hex_cmd[2] = ACTUAL_POSITION_H[0];
        // hex_cmd[3] = ACTUAL_POSITION_H[1];
        // hex_cmd[4] = 0x00;
        // hex_cmd[5] = 0x02;

        // calculate_crc();

        // serial_.write(hex_cmd, 8);

        // // read_hex(7);
        // std::string line = serial_.read(9);
        // // print_hex_cmd();
        // // convert string to hex
        // for (uint8_t i = 0; i < uint8_t(line.size()); i++)
        // {
        //     receive_hex[i] = uint8_t(line[i]);
        //     // printf("rec %d, %02x\n", i, receive_hex[i]);
        // }

        // // crc check of received data
        // if (crc16(receive_hex, 9) != 0)
        // {
        //     printf("crc checking error get postion\n");
        //     // return 1;
        // }
        // // print_rec_hex(8);

        return receive_hex[6] + (receive_hex[5] << 8) + (receive_hex[4] << 16) + (receive_hex[3] << 24);
    }

    /**
     * @brief Read data form the motor
     *        - position in counts, one rotation has about 4090 counts
     *        - rpm
     *        - torque in 0.1 A
     *        - Error message
     * @return 0 if ok, 1 if crc read error
     */
    uint8_t read_motor()
    {
        // memset(hex_cmd, 0, sizeof(hex_cmd));
        hex_cmd[0] = ID;
        hex_cmd[1] = READ;
        hex_cmd[2] = ACTUAL_POSITION_H[0];
        hex_cmd[3] = ACTUAL_POSITION_H[1];
        hex_cmd[4] = 0x00;
        hex_cmd[5] = 0x05;

        calculate_crc();

        for (int i = 0; i < int(sizeof(hex_cmd) / sizeof(uint8_t)); i++)
        {
            serial_.WriteByte(hex_cmd[i]);
        }
        serial_.DrainWriteBuffer();

        // read_hex(7);
        std::string read_str;

        try
        {
            serial_.Read(read_str, 15, timeout_ms_);
        }
        catch (const LibSerial::ReadTimeout &)
        {
            std::cerr << "The serial read call has timed out." << std::endl;
        }

        // print_hex_cmd();
        // convert string to hex
        for (uint8_t i = 0; i < uint8_t(read_str.size()); i++)
        {
            receive_hex[i] = uint8_t(read_str[i]);
            // printf("rec %d, %02x\n", i, receive_hex[i]);
        }

        // crc check of received data
        if (crc16(receive_hex, 15) != 0)
        {
            serial_.FlushIOBuffers();
            // printf("crc checking error read_motor\n");
            return 1;
        }
        // print_rec_hex();

        return 0;
    }

    /**
     * @return 0 when OK. 1 if crc error
     */
    uint8_t enable()
    {
        memset(hex_cmd, 0, sizeof(hex_cmd));
        hex_cmd[0] = ID;
        hex_cmd[1] = WRITE;
        hex_cmd[2] = CONTROL_REG[0];
        hex_cmd[3] = CONTROL_REG[1];
        hex_cmd[4] = ENABLE[0];
        hex_cmd[5] = ENABLE[1];

        calculate_crc();

        for (int i = 0; i < int(sizeof(hex_cmd) / sizeof(uint8_t)); i++)
        {
            serial_.WriteByte(hex_cmd[i]);
        }
        serial_.DrainWriteBuffer();

        if (read_hex(8))
            return 1;
        return 0;
    }

    /**
     * @brief when motor disabled wheel can spin freely but still can read the rpm
     * @return 0 when OK. 1 if crc error
     */
    uint8_t disable()
    {
        // memset(hex_cmd, 0, sizeof(hex_cmd));
        hex_cmd[0] = ID;
        hex_cmd[1] = WRITE;
        hex_cmd[2] = CONTROL_REG[0];
        hex_cmd[3] = CONTROL_REG[1];
        hex_cmd[4] = DISABLE[0];
        hex_cmd[5] = DISABLE[1];

        calculate_crc();

        for (int i = 0; i < int(sizeof(hex_cmd) / sizeof(uint8_t)); i++)
        {
            serial_.WriteByte(hex_cmd[i]);
        }
        serial_.DrainWriteBuffer();

        if (read_hex(8))
            return 1;
        return 0;
    }

    uint8_t set_vel_mode()
    {
        // memset(hex_cmd, 0, sizeof(hex_cmd));
        hex_cmd[0] = ID;
        hex_cmd[1] = WRITE;
        hex_cmd[2] = OPERATING_MODE[0];
        hex_cmd[3] = OPERATING_MODE[1];
        hex_cmd[4] = VEL_MODE[0];
        hex_cmd[5] = VEL_MODE[1];

        calculate_crc();

        for (int i = 0; i < int(sizeof(hex_cmd) / sizeof(uint8_t)); i++)
        {
            serial_.WriteByte(hex_cmd[i]);
        }
        serial_.DrainWriteBuffer();

        read_hex(8);
        return 0;
    }

    /**
     * @param acc_time_ms acceleration time in ms eg. 500
     * @return 0 when OK. 1 if crc error
     */
    uint8_t set_acc_time(uint16_t acc_time_ms)
    {
        // memset(hex_cmd, 0, sizeof(hex_cmd));
        hex_cmd[0] = ID;
        hex_cmd[1] = WRITE;
        hex_cmd[2] = SET_ACC_TIME[0];
        hex_cmd[3] = SET_ACC_TIME[1];
        hex_cmd[4] = (acc_time_ms >> 8) & 0xFF;
        hex_cmd[5] = acc_time_ms & 0xFF;

        calculate_crc();

        for (int i = 0; i < int(sizeof(hex_cmd) / sizeof(uint8_t)); i++)
        {
            serial_.WriteByte(hex_cmd[i]);
        }
        serial_.DrainWriteBuffer();

        if (read_hex(8))
            return 1;
        return 0;
    }

    /**
     * @param decc_time_ms decceleration time in ms eg. 500
     * @return 0 when OK. 1 if crc error
     */
    uint8_t set_decc_time(uint16_t decc_time_ms)
    {
        // memset(hex_cmd, 0, sizeof(hex_cmd));
        hex_cmd[0] = ID;
        hex_cmd[1] = WRITE;
        hex_cmd[2] = SET_DECC_TIME[0];
        hex_cmd[3] = SET_DECC_TIME[1];
        hex_cmd[4] = (decc_time_ms >> 8) & 0xFF;
        hex_cmd[5] = decc_time_ms & 0xFF;

        calculate_crc();

        for (int i = 0; i < int(sizeof(hex_cmd) / sizeof(uint8_t)); i++)
        {
            serial_.WriteByte(hex_cmd[i]);
        }
        serial_.DrainWriteBuffer();

        if (read_hex(8))
            return 1;
        return 0;
    }

    /**
     * @param vel_smoothing_factor Default: 1000, Range: 0-30000
     * @return 0 when OK. 1 if crc error
     */
    uint8_t set_vel_smoothing(uint16_t vel_smoothing_factor)
    {
        // memset(hex_cmd, 0, sizeof(hex_cmd));
        hex_cmd[0] = ID;
        hex_cmd[1] = WRITE;
        hex_cmd[2] = SET_VEL_SMOOTHING[0];
        hex_cmd[3] = SET_VEL_SMOOTHING[1];
        hex_cmd[4] = (vel_smoothing_factor >> 8) & 0xFF;
        hex_cmd[5] = vel_smoothing_factor & 0xFF;

        calculate_crc();

        for (int i = 0; i < int(sizeof(hex_cmd) / sizeof(uint8_t)); i++)
        {
            serial_.WriteByte(hex_cmd[i]);
        }
        serial_.DrainWriteBuffer();

        if (read_hex(8))
            return 1;
        return 0;
    }

    /**
     * @param feedforward_output_smoothing_factor Default: 100, Range: 0-30000
     * @return 0 when OK. 1 if crc error
     */
    uint8_t set_feedforward_output_smoothing(uint16_t feedforward_output_smoothing_factor)
    {
        // memset(hex_cmd, 0, sizeof(hex_cmd));
        hex_cmd[0] = ID;
        hex_cmd[1] = WRITE;
        hex_cmd[2] = SET_FORWARD_OUTPUT_SMOOTHING[0];
        hex_cmd[3] = SET_FORWARD_OUTPUT_SMOOTHING[1];
        hex_cmd[4] = (feedforward_output_smoothing_factor >> 8) & 0xFF;
        hex_cmd[5] = feedforward_output_smoothing_factor & 0xFF;

        calculate_crc();

        for (int i = 0; i < int(sizeof(hex_cmd) / sizeof(uint8_t)); i++)
        {
            serial_.WriteByte(hex_cmd[i]);
        }
        serial_.DrainWriteBuffer();

        if (read_hex(8))
            return 1;
        return 0;
    }

    /**
     * @param proportional_gain Speed Proportional Gain. Default: 500, Range: 0-30000
     * @return 0 when OK. 1 if crc error
     */
    uint8_t set_kp(uint16_t proportional_gain)
    {
        // memset(hex_cmd, 0, sizeof(hex_cmd));
        hex_cmd[0] = ID;
        hex_cmd[1] = WRITE;
        hex_cmd[2] = SET_KP[0];
        hex_cmd[3] = SET_KP[1];
        hex_cmd[4] = (proportional_gain >> 8) & 0xFF;
        hex_cmd[5] = proportional_gain & 0xFF;

        calculate_crc();

        for (int i = 0; i < int(sizeof(hex_cmd) / sizeof(uint8_t)); i++)
        {
            serial_.WriteByte(hex_cmd[i]);
        }
        serial_.DrainWriteBuffer();

        if (read_hex(8))
            return 1;
        return 0;
    }

    /**
     * @param integral_gain Speed Integral Gain. Default: 100, Range: 0-30000
     * @return 0 when OK. 1 if crc error
     */
    uint8_t set_ki(uint16_t integral_gain)
    {
        // memset(hex_cmd, 0, sizeof(hex_cmd));
        hex_cmd[0] = ID;
        hex_cmd[1] = WRITE;
        hex_cmd[2] = SET_KI[0];
        hex_cmd[3] = SET_KI[1];
        hex_cmd[4] = (integral_gain >> 8) & 0xFF;
        hex_cmd[5] = integral_gain & 0xFF;

        calculate_crc();

        for (int i = 0; i < int(sizeof(hex_cmd) / sizeof(uint8_t)); i++)
        {
            serial_.WriteByte(hex_cmd[i]);
        }
        serial_.DrainWriteBuffer();

        if (read_hex(8))
            return 1;
        return 0;
    }

    /**
     * @param forward_gain Speed Forward Gain. Default: 1000, Range: 0-30000
     * @return 0 when OK. 1 if crc error
     */
    uint8_t set_kf(uint16_t forward_gain)
    {
        // memset(hex_cmd, 0, sizeof(hex_cmd));
        hex_cmd[0] = ID;
        hex_cmd[1] = WRITE;
        hex_cmd[2] = SET_KF[0];
        hex_cmd[3] = SET_KF[1];
        hex_cmd[4] = (forward_gain >> 8) & 0xFF;
        hex_cmd[5] = forward_gain & 0xFF;

        calculate_crc();

        for (int i = 0; i < int(sizeof(hex_cmd) / sizeof(uint8_t)); i++)
        {
            serial_.WriteByte(hex_cmd[i]);
        }
        serial_.DrainWriteBuffer();

        if (read_hex(8))
            return 1;
        return 0;
    }

    /**
     * @return The ini tial speed when moti on begins.
     */
    uint8_t initial_speed(uint16_t rpm)
    {
        // memset(hex_cmd, 0, sizeof(hex_cmd));
        hex_cmd[0] = ID;
        hex_cmd[1] = WRITE;
        hex_cmd[2] = INITIAL_SPEED[0];
        hex_cmd[3] = INITIAL_SPEED[1];
        hex_cmd[4] = (rpm >> 8) & 0xFF;
        hex_cmd[5] = rpm & 0xFF;

        calculate_crc();

        for (int i = 0; i < int(sizeof(hex_cmd) / sizeof(uint8_t)); i++)
        {
            serial_.WriteByte(hex_cmd[i]);
        }
        serial_.DrainWriteBuffer();

        if (read_hex(8))
            return 1;
        return 0;
    }

    /**
     * @return Max operating speed of motor.
     */
    uint8_t max_speed(uint16_t rpm)
    {
        // memset(hex_cmd, 0, sizeof(hex_cmd));
        hex_cmd[0] = ID;
        hex_cmd[1] = WRITE;
        hex_cmd[2] = MAX_SPEED[0];
        hex_cmd[3] = MAX_SPEED[1];
        hex_cmd[4] = (rpm >> 8) & 0xFF;
        hex_cmd[5] = rpm & 0xFF;

        calculate_crc();

        for (int i = 0; i < int(sizeof(hex_cmd) / sizeof(uint8_t)); i++)
        {
            serial_.WriteByte(hex_cmd[i]);
        }
        serial_.DrainWriteBuffer();

        if (read_hex(8))
            return 1;
        return 0;
    }

public:
    LibSerial::SerialPort serial_;
    int timeout_ms_;
};

class ZLAC8015Comm : public BaseComm
{
public:
    ZLAC8015Comm() {}

    ~ZLAC8015Comm() {}

    bool ConnectComm(std::string port, int baud_rate, int timeout_ms)
    {
        bool success = true;
        success = success && motorL.begin(port + "_left", baud_rate, 0x04);
        success = success && motorR.begin(port + "_right", baud_rate, 0x04);
        motorL.timeout_ms_ = timeout_ms;
        motorR.timeout_ms_ = timeout_ms;

        // left motor init
        motorL.set_vel_mode();
        motorL.enable();
        motorL.set_acc_time(500);
        motorL.set_decc_time(500);
        motorL.set_ki(100);
        motorL.set_kp(500);
        motorL.set_kf(1000);
        motorL.max_speed(80);
        motorL.set_feedforward_output_smoothing(100);
        motorL.set_vel_smoothing(1000);

        // right motor init
        motorR.set_vel_mode();
        motorR.enable();
        motorR.set_acc_time(500);
        motorR.set_decc_time(500);
        motorR.set_ki(100);
        motorR.set_kp(500);
        motorR.set_kf(1000);
        motorR.max_speed(80);
        motorR.set_feedforward_output_smoothing(100);
        motorR.set_vel_smoothing(1000);

        return success;
    }

    bool DisconnectComm()
    {
        motorL.disable();
        motorR.disable();
        motorL.serial_.Close();
        motorR.serial_.Close();;

        return true;
    }

    bool connected()
    {
        return motorL.serial_.IsOpen() && motorR.serial_.IsOpen();
    }

    void DriveCommand(double left_motor_rpm, double right_motor_rpm)
    {
        motorL.set_rpm(left_motor_rpm);
        motorR.set_rpm(right_motor_rpm);
    }

    bool ReadRPM(double &left_encoder_rpm, double &right_encoder_rpm)
    {
        if (!motorL.read_motor())
            left_encoder_rpm = motorL.get_rpm();

        if (!motorR.read_motor())
            right_encoder_rpm = motorR.get_rpm();

        return true;
    }

private:
    ZLAC8015 motorL;
    ZLAC8015 motorR;
};

#endif // ZLAC8015_COMMS_HPP