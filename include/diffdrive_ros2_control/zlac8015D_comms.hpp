#ifndef ZLAC8015D_COMMS_HPP
#define ZLAC8015D_COMMS_HPP

// #include "diffdrive_ros2_control/base_comms.hpp"

// #include <stdlib.h>
// #include <stdio.h>
// #include <time.h>
// #include <stdint.h>
// #include <cstring>
// #include <vector>
// #include <cstdio>
// #include <chrono>

// // OS Specific sleep
// #ifdef _WIN32
// #include <windows.h>
// #else
// #include <unistd.h>
// #endif

// /*================= CRC Check ==============*/

// constexpr unsigned short crc16_table[] = {
//     0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
//     0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
//     0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
//     0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
//     0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
//     0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
//     0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
//     0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
//     0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
//     0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
//     0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
//     0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
//     0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
//     0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
//     0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
//     0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
//     0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
//     0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
//     0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
//     0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
//     0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
//     0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
//     0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
//     0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
//     0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
//     0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
//     0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
//     0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
//     0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
//     0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
//     0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
//     0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040};

// static inline unsigned short crc16(const unsigned char *data, unsigned short len)
// {
//     unsigned char nTemp;
//     unsigned short wCRCWord = 0xFFFF;

//     while (len--)
//     {
//         nTemp = *data++ ^ wCRCWord;
//         wCRCWord >>= 8;
//         wCRCWord ^= crc16_table[nTemp];
//     }
//     return wCRCWord;
// }

// /*================= CRC Check End ==============*/

// class Zlac8015DComm : public BaseComm
// {
// public:
//     Zlac8015DComm() {}
//     ~Zlac8015DComm() {}
// };

// class ZLAC8015
// {

// protected:
//     std::chrono::time_point<std::chrono::steady_clock> start, end;

//     uint8_t hex_cmd[8] = {0};
//     uint8_t receive_hex[15] = {0};
//     uint8_t ID = 0x00;
//     const uint8_t READ = 0x03;
//     const uint8_t WRITE = 0x06;
//     const uint8_t MULTI_WRITE = 0x10;
//     const uint8_t CONTROL_REG[2] = {0X20, 0X31};
//     const uint8_t ENABLE[2] = {0x00, 0X08};
//     const uint8_t DISABLE[2] = {0x00, 0X07};
//     const uint8_t OPERATING_MODE[2] = {0X20, 0X32};
//     const uint8_t VEL_MODE[2] = {0x00, 0X03};
//     const uint8_t SET_RPM[2] = {0x20, 0X3A};
//     const uint8_t GET_RPM[2] = {0x20, 0X2C};
//     const uint8_t SET_ACC_TIME[2] = {0x20, 0X81};
//     const uint8_t SET_DECC_TIME[2] = {0x20, 0X83};
//     const uint8_t SET_KP[2] = {0x20, 0X1D};
//     const uint8_t SET_KI[2] = {0x20, 0X1E};
//     const uint8_t INITIAL_SPEED[2] = {0X20, 0X08};
//     const uint8_t MAX_SPEED[2] = {0X20, 0X0A};
//     const uint8_t ACTUAL_POSITION_H[2] = {0X20, 0X2A};
//     const uint8_t ACTUAL_POSITION_L[2] = {0X20, 0X2B};

//     // HELPER Functions *************************************************************

//     void sleep(unsigned long milliseconds)
//     {
// #ifdef _WIN32
//         Sleep(milliseconds); // 100 ms
// #else
//         usleep(milliseconds * 1000); // 100 ms
// #endif
//     }

//     /**
//      * @brief calculates the crc and stores it in the hex_cmd array, so there is no return value
//      */
//     void calculate_crc()
//     {
//         // calculate crc and append to hex cmd
//         unsigned short result = crc16(hex_cmd, sizeof(hex_cmd) - 2);
//         hex_cmd[sizeof(hex_cmd) - 2] = result & 0xFF;
//         hex_cmd[sizeof(hex_cmd) - 1] = (result >> 8) & 0xFF;
//     }

//     /**
//      * @brief reads from the serial port and saves the string into the receive_hex array
//      * @param num_bytes how many bytes to read from the buffer
//      * @return return 0 when OK, 1 if crc error
//      */
//     uint8_t read_hex(uint8_t num_bytes)
//     {
//         serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
//         _serial.setTimeout(timeout);

//         std::string line = _serial.read(num_bytes);

//         // convert string to hex
//         printf("receive_hex:");
//         for (uint8_t i = 0; i < uint8_t(line.size()); i++)
//         {
//             receive_hex[i] = uint8_t(line[i]);
//             printf(" %02x", receive_hex[i]);
//         }
//         // crc check of received data
//         if (crc16(receive_hex, num_bytes) != 0)
//         {
//             printf("crc checking error\n");
//             return 1;
//         }
//         return 0;
//     }

//     /**
//      * @brief print the hex command for debugging
//      */
//     void print_hex_cmd() const
//     {
//         // print
//         for (int i = 0; i < 8; i++)
//         {
//             printf("%d, %02x\n", i, hex_cmd[i]);
//         }
//     }

//     /**
//      * @brief print received hex for debugging
//      */
//     void print_rec_hex() const
//     {
//         // print
//         for (int i = 0; i < 8; i++)
//         {
//             printf("rec: %d, %02x\n", i, receive_hex[i]);
//         }
//     }

// public:
//     /**
//      * @brief open serial port communication
//      * @param port COM port eg. "/dev/ttyUSB0"
//      * @param baudRate default baudrate is 115200
//      * @param _ID Set the modbus ID of the motor driver in HEX, default 0x00
//      */
//     void begin(std::string port, int baudrate, uint8_t ID)
//     {
//         this->ID = ID;
//         _serial.setPort(port);
//         _serial.setBaudrate(baudrate);
//         serial::Timeout timeout = serial::Timeout::simpleTimeout(100);

//         _serial.setTimeout(timeout);

//         _serial.open();
//         _serial.flushInput();
//         printf("%d: SERIAL OK!", ID);
//     }

//     /**
//      * @param rpm
//      * @param side
//      * @return alwasy 0
//      */
//     uint8_t set_rpm(int16_t rpm, std::string side)
//     {
//         memset(hex_cmd, 0, sizeof(hex_cmd));
//         hex_cmd[0] = ID;
//         hex_cmd[1] = WRITE;
//         hex_cmd[2] = 0x20;
//         if (side == "LEFT")
//         {
//             hex_cmd[3] = 0x88;
//         }
//         else if (side == "RIGHT")
//         {
//             hex_cmd[3] = 0x89;
//         }
//         // for left motor, 0x88

//         hex_cmd[4] = (rpm >> 8) & 0xFF;
//         hex_cmd[5] = rpm & 0xFF;

//         calculate_crc();

//         // TODO isn't save, to continue reading infinite in case of error
//         // do // repeat sending and read command as long as it has crc error
//         // {
//         //     _serial.write(hex_cmd, 8);
//         //     // print_rec_hex();
//         // } while (read_hex(8));
//         _serial.write(hex_cmd, 8);
//         read_hex(8);
//         return 0;
//     }

//     /**
//      * @param rpm
//      * @return alwasy 0
//      */
//     uint8_t set_sync_rpm(int16_t rpm)
//     {
//         memset(hex_cmd, 0, sizeof(hex_cmd));
//         hex_cmd[0] = ID;
//         hex_cmd[1] = MULTI_WRITE;
//         hex_cmd[2] = 0x20;
//         hex_cmd[3] = 0x88;
//         hex_cmd[4] = 0x00;
//         hex_cmd[5] = 0x02;
//         hex_cmd[6] = 0x04;

//         hex_cmd[7] = (rpm >> 8) & 0xFF;
//         hex_cmd[8] = rpm & 0xFF;

//         hex_cmd[9] = (rpm >> 8) & 0xFF;
//         hex_cmd[10] = rpm & 0xFF;

//         // TODO isn't save, to continue reading infinite in case of error
//         // do // repeat sending and read command as long as it has crc error
//         // {
//         //     _serial.write(hex_cmd, 8);
//         //     // print_rec_hex();
//         // } while (read_hex(8));
//         calculate_crc();

//         printf("serial.write :");
//         for (int i = 0; i < 13; ++i)
//         {
//             printf("%02X ", hex_cmd[i]);
//         }
//         _serial.write(hex_cmd, 13);
//         read_hex(8);
//         return 0;
//     }

//     /**
//      * @return rpm measured from wheel
//      */
//     float get_rpm()
//     {
//         memset(hex_cmd, 0, sizeof(hex_cmd));
//         hex_cmd[0] = ID;
//         hex_cmd[1] = READ;
//         hex_cmd[2] = GET_RPM[0];
//         hex_cmd[3] = GET_RPM[1];

//         hex_cmd[4] = 0x00;
//         hex_cmd[5] = 0x01;

//         calculate_crc();
//         // print_hex_cmd();

//         // TODO isn't save, to continue reading infinite in case of error
//         // do // repeat sending and read command as long as it has crc error
//         // {
//         //     _serial.write(hex_cmd, 8);
//         //     // print_rec_hex();
//         // } while (read_hex(7) );

//         _serial.write(hex_cmd, 8);
//         read_hex(7);
//         int16_t rpm_tenth = receive_hex[8] + (receive_hex[7] << 8);
//         return (float)rpm_tenth / 10.0f;
//     }

//     /**
//      * @return Actual torque feedback, unit: A
//      */
//     float get_torque()
//     {
//         int16_t torque = receive_hex[10] + (receive_hex[9] << 8);
//         return (float)torque / 10.0f;
//     }

//     /**
//      * @return Error feedback,
//      *          0000h: no error;
//      *          0001h: over-voltage;
//      *          0002h: under-voltage;
//      *          0004h: over-current;
//      *          0008h: overload;
//      *          0010h: current is out of tolerance;
//      *          0020h: encoder is out of tolerance;
//      *          0040h: speed is out of tolerance;
//      *          0080h: reference voltage error;
//      *          0100h: EEPROM read and write error;
//      *          0200h: Hall error;
//      *          0400h: motor temperature is too high.
//      */
//     uint16_t get_error()
//     {
//         memset(hex_cmd, 0, sizeof(hex_cmd));
//         hex_cmd[0] = 0x03;
//         hex_cmd[1] = 0x03;
//         hex_cmd[2] = 0x20;
//         hex_cmd[3] = 0x00;
//         hex_cmd[4] = 0x01;
//         calculate_crc();
//         _serial.write(hex_cmd, 7);
//         printf("serial.write :");
//         for (int i = 0; i < 7; ++i)
//         {
//             printf("%02X ", hex_cmd[i]);
//         }

//         printf("\nresponse:\n");
//         if (read_hex(7))
//         {
//             printf("\nNo response.... Nah\n");
//             return 1;
//         }
//         return 0;
//         // return receive_hex[12] + (receive_hex[11] << 8);
//     }

//     /**
//      * @return Actual position feedback, unit: counts
//      */
//     int32_t get_position()
//     {
//         // // memset(hex_cmd, 0, sizeof(hex_cmd));
//         // hex_cmd[0] = ID;
//         // hex_cmd[1] = READ;
//         // hex_cmd[2] = ACTUAL_POSITION_H[0];
//         // hex_cmd[3] = ACTUAL_POSITION_H[1];
//         // hex_cmd[4] = 0x00;
//         // hex_cmd[5] = 0x02;

//         // calculate_crc();

//         // _serial.write(hex_cmd, 8);

//         // // read_hex(7);
//         // std::string line = _serial.read(9);
//         // // print_hex_cmd();
//         // // convert string to hex
//         // for (uint8_t i = 0; i < uint8_t(line.size()); i++)
//         // {
//         //     receive_hex[i] = uint8_t(line[i]);
//         //     // printf("rec %d, %02x\n", i, receive_hex[i]);
//         // }

//         // // crc check of received data
//         // if (crc16(receive_hex, 9) != 0)
//         // {
//         //     printf("crc checking error get postion\n");
//         //     // return 1;
//         // }
//         // // print_rec_hex(8);

//         return receive_hex[6] + (receive_hex[5] << 8) + (receive_hex[4] << 16) + (receive_hex[3] << 24);
//     }

//     /**
//      * @brief Read data form the motor
//      *        - position in counts, one rotation has about 4090 counts
//      *        - rpm
//      *        - torque in 0.1 A
//      *        - Error message
//      * @return 0 if ok, 1 if crc read error
//      */
//     uint8_t read_motor()
//     {
//         // memset(hex_cmd, 0, sizeof(hex_cmd));
//         hex_cmd[0] = ID;
//         hex_cmd[1] = READ;
//         hex_cmd[2] = ACTUAL_POSITION_H[0];
//         hex_cmd[3] = ACTUAL_POSITION_H[1];
//         hex_cmd[4] = 0x00;
//         hex_cmd[5] = 0x05;

//         calculate_crc();

//         _serial.write(hex_cmd, 8);

//         // read_hex(7);
//         std::string line = _serial.read(15);
//         // print_hex_cmd();
//         // convert string to hex
//         for (uint8_t i = 0; i < uint8_t(line.size()); i++)
//         {
//             receive_hex[i] = uint8_t(line[i]);
//             // printf("rec %d, %02x\n", i, receive_hex[i]);
//         }

//         // crc check of received data
//         // if (crc16(receive_hex, 15) != 0)
//         // {
//         //     _serial.flush();
//         //     // printf("crc checking error read_motor\n");
//         //     return 1;
//         // }
//         // print_rec_hex();

//         return 0;
//     }

//     /**
//      * @return 0 when OK. 1 if crc error
//      */
//     uint8_t enable()
//     {
//         memset(hex_cmd, 0, sizeof(hex_cmd));
//         hex_cmd[0] = ID;
//         hex_cmd[1] = WRITE;
//         hex_cmd[2] = 0x20;
//         hex_cmd[3] = 0x0E;

//         hex_cmd[4] = ENABLE[0];
//         hex_cmd[5] = ENABLE[1];

//         calculate_crc();
//         printf("serial.write :");
//         for (int i = 0; i < 8; ++i)
//         {
//             printf("%02X ", hex_cmd[i]);
//         }
//         _serial.write(hex_cmd, 8);
//         printf("\nresponse:\n");
//         if (read_hex(8))
//         {
//             printf("\nNo response.... Nah\n");
//             return 1;
//         }
//         return 0;
//     }

//     /**
//      * @brief when motor disabled wheel can spin freely but still can read the rpm
//      * @return 0 when OK. 1 if crc error
//      */
//     uint8_t disable()
//     {
//         memset(hex_cmd, 0, sizeof(hex_cmd));
//         hex_cmd[0] = ID;
//         hex_cmd[1] = WRITE;
//         hex_cmd[2] = 0x20;
//         hex_cmd[3] = 0x31;

//         hex_cmd[4] = DISABLE[0];
//         hex_cmd[5] = DISABLE[1];

//         calculate_crc();
//         printf("serial.write :");
//         for (int i = 0; i < 8; ++i)
//         {
//             printf("%02X ", hex_cmd[i]);
//         }
//         _serial.write(hex_cmd, 8);
//         if (read_hex(8))
//         {
//             printf("\nNo response.... Nah\n");
//             return 1;
//         }
//         return 0;
//     }

//     uint8_t set_vel_mode()
//     {
//         // memset(hex_cmd, 0, sizeof(hex_cmd));
//         hex_cmd[0] = ID;
//         hex_cmd[1] = WRITE;

//         // hex_cmd[2] = OPERATING_MODE[0];
//         // hex_cmd[3] = OPERATING_MODE[1];
//         hex_cmd[2] = 0x20;
//         hex_cmd[3] = 0x0D;
//         hex_cmd[4] = VEL_MODE[0];
//         hex_cmd[5] = VEL_MODE[1];

//         calculate_crc();
//         _serial.write(hex_cmd, 8);
//         printf("serial.write :");
//         for (int i = 0; i < 8; ++i)
//         {
//             printf("%02X ", hex_cmd[i]);
//         }
//         printf("\nresponse:\n");
//         if (read_hex(8))
//         {
//             printf("\nNo response.... Nah\n");
//             return 1;
//         }
//         return 0;
//     }

//     /**
//      * @param acc_time_ms acceleration time in ms eg. 500
//      * @return 0 when OK. 1 if crc error
//      */
//     uint8_t set_acc_time(uint16_t acc_time_ms)
//     {
//         // memset(hex_cmd, 0, sizeof(hex_cmd));
//         hex_cmd[0] = ID;
//         hex_cmd[1] = WRITE;
//         hex_cmd[2] = SET_ACC_TIME[0];
//         hex_cmd[3] = SET_ACC_TIME[1];

//         hex_cmd[4] = (acc_time_ms >> 8) & 0xFF;
//         hex_cmd[5] = acc_time_ms & 0xFF;

//         calculate_crc();
//         _serial.write(hex_cmd, 8);
//         if (read_hex(8))
//             return 1;
//         return 0;
//     }

//     /**
//      * @param decc_time_ms decceleration time in ms eg. 500
//      * @return 0 when OK. 1 if crc error
//      */
//     uint8_t set_decc_time(uint16_t decc_time_ms)
//     {
//         // memset(hex_cmd, 0, sizeof(hex_cmd));
//         hex_cmd[0] = ID;
//         hex_cmd[1] = WRITE;
//         hex_cmd[2] = SET_DECC_TIME[0];
//         hex_cmd[3] = SET_DECC_TIME[1];

//         hex_cmd[4] = (decc_time_ms >> 8) & 0xFF;
//         hex_cmd[5] = decc_time_ms & 0xFF;

//         calculate_crc();
//         _serial.write(hex_cmd, 8);
//         if (read_hex(8))
//             return 1;
//         return 0;
//     }

//     /**
//      * @param proportional_gain Speed Proportional Gain. Default: 500
//      * @return 0 when OK. 1 if crc error
//      */
//     uint8_t set_kp(uint16_t proportional_gain)
//     {
//         // memset(hex_cmd, 0, sizeof(hex_cmd));
//         hex_cmd[0] = ID;
//         hex_cmd[1] = WRITE;
//         hex_cmd[2] = SET_KP[0];
//         hex_cmd[3] = SET_KP[1];

//         hex_cmd[4] = (proportional_gain >> 8) & 0xFF;
//         hex_cmd[5] = proportional_gain & 0xFF;

//         calculate_crc();
//         _serial.write(hex_cmd, 8);
//         if (read_hex(8))
//             return 1;
//         return 0;
//     }

//     /**
//      * @param integral_gain Speed Integral Gain. Default: 100
//      * @return 0 when OK. 1 if crc error
//      */
//     uint8_t set_ki(uint16_t integral_gain)
//     {
//         // memset(hex_cmd, 0, sizeof(hex_cmd));
//         hex_cmd[0] = ID;
//         hex_cmd[1] = WRITE;
//         hex_cmd[2] = SET_KI[0];
//         hex_cmd[3] = SET_KI[1];

//         hex_cmd[4] = (integral_gain >> 8) & 0xFF;
//         hex_cmd[5] = integral_gain & 0xFF;

//         calculate_crc();
//         _serial.write(hex_cmd, 8);
//         if (read_hex(8))
//             return 1;
//         return 0;
//     }

//     /**
//      * @return The ini tial speed when moti on begins.
//      */
//     uint8_t initial_speed(uint16_t rpm)
//     {
//         // memset(hex_cmd, 0, sizeof(hex_cmd));
//         hex_cmd[0] = ID;
//         hex_cmd[1] = WRITE;
//         hex_cmd[2] = INITIAL_SPEED[0];
//         hex_cmd[3] = INITIAL_SPEED[1];

//         hex_cmd[4] = (rpm >> 8) & 0xFF;
//         hex_cmd[5] = rpm & 0xFF;

//         calculate_crc();
//         _serial.write(hex_cmd, 8);
//         if (read_hex(8))
//             return 1;
//         return 0;
//     }

//     /**
//      * @return Max operating speed of motor.
//      */
//     uint8_t max_speed(uint16_t rpm)
//     {
//         // memset(hex_cmd, 0, sizeof(hex_cmd));
//         hex_cmd[0] = ID;
//         hex_cmd[1] = WRITE;
//         hex_cmd[2] = MAX_SPEED[0];
//         hex_cmd[3] = MAX_SPEED[1];

//         hex_cmd[4] = (rpm >> 8) & 0xFF;
//         hex_cmd[5] = rpm & 0xFF;

//         calculate_crc();
//         _serial.write(hex_cmd, 8);
//         if (read_hex(8))
//             return 1;
//         return 0;
//     }

// private:
//     LibSerial::SerialPort serial_;
//     int timeout_ms_;
// };

#endif // ZLAC8015D_COMMS_HPP