#pragma once

#if defined(_WIN32)
#include <windows.h>

#elif defined(__ANDROID__)
#include <filesystem>

#elif defined(__linux__)
#include <filesystem>
#include <termio.h>
#include <sys/ioctl.h>

#include <linux/limits.h>
#include <sys/time.h>
#include <unistd.h>

#elif defined(__APPLE__)
#include <dirent.h>

#else
#error "Unspported OS"
#endif

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <fcntl.h>
#include <cstring>
#include <algorithm>
#include <ctime>

#include "Controller.h"

class SerialPortConnector {
public:
    SerialPortConnector();
    ~ SerialPortConnector() {
#if defined(__WIN32__)
        if (hComm != nullptr) CloseHandle(hComm);
        delete static_cast<HANDLE*>(hComm);
#elif defined(__linux__)
        if (fd != nullptr && fcntl(*fd, F_GETFD) != -1) close(*fd);
#endif
        delete fd;
        delete selectedPort;
        delete baudrate;
    };

    // std::vector<Controller> controllers;
    std::vector<std::unique_ptr<Controller>> controllers;

    std::vector<std::string> allPorts;
    int* fd;
    void* hComm;

    std::string* selectedPort;
    unsigned int* baudrate;

    std::vector<std::string> getPorts();
    bool connectPort(std::string port, unsigned int speed);
    void disconnect();
    Controller& addController(uint8_t id);
    Controller& getController(uint8_t id);
};

namespace Address {
    constexpr uint8_t ID                         = 0x01;
    constexpr uint8_t ADDRESS_ERROR              = 0x02;
    constexpr uint8_t COMMUNICATION_MODE         = 0x03;
    constexpr uint8_t BAUDRATE                   = 0x04;
    constexpr uint8_t WATCHDOG                   = 0x08;

    constexpr uint8_t VOLTAGE_NOW                = 0x09;
    constexpr uint8_t TEMPERATURE_NOW            = 0x0B;

    constexpr uint8_t VOLTAGE_MAX                = 0x0C;
    constexpr uint8_t VOLTAGE_MIN                = 0x0E;
    constexpr uint8_t TEMPERATURE_MAX            = 0x0F;

    constexpr uint8_t MOTOR_TYPE                 = 0x10;
    constexpr uint8_t MOTOR_ENCODER_DIR          = 0x11;
    constexpr uint8_t MOTOR_ENCODER_CPR        = 0x12;
    constexpr uint8_t ELECTRONIC_ANGLE           = 0x14;
    constexpr uint8_t MOTOR_GEAR                 = 0x16;
    constexpr uint8_t MOTOR_LEAD_ANGLE_MODE      = 0x18;
    constexpr uint8_t MOTOR_FORCE_ANGLE_LEVEL    = 0x19;
    constexpr uint8_t MOTOR_ELECTRICAL_ZERO      = 0x1A;
    constexpr uint8_t MOTOR_HALLSENSOR_SETTING   = 0x1C;

    constexpr uint8_t MOTOR_D_INDUCTANCE_CONSTANT     = 0x1D;
    constexpr uint8_t MOTOR_Q_INDUCTANCE_CONSTANT     = 0x1E;
    constexpr uint8_t MOTOR_FLUX_LINKAGE_CONSTANT     = 0x1F;
    constexpr uint8_t MOTOR_RESISTANCE_CONSTANT       = 0x20;
    constexpr uint8_t MOTOR_INDUCTANCE_CONSTANT       = 0x22;
    constexpr uint8_t MOTOR_TORQUE_CONSTANT           = 0x24;
    constexpr uint8_t MOTOR_MOMENT_OF_INERTIA_CONSTANT= 0x26;
    constexpr uint8_t MOTOR_RATED_SPEED               = 0x28;
    constexpr uint8_t MOTOR_RATED_CURRENT             = 0x2A;
    constexpr uint8_t MOTOR_RATED_VOLTAGE             = 0x2C;

    constexpr uint8_t POSITION_TOTAL_GAIN             = 0x2D;
    constexpr uint8_t VELOCITY_TOTAL_GAIN             = 0x2E;
    constexpr uint8_t CURRENT_TOTAL_GAIN              = 0x2F;
    constexpr uint8_t POSITION_P_GAIN                 = 0x30;
    constexpr uint8_t VELOCITY_P_GAIN                 = 0x32;
    constexpr uint8_t VELOCITY_I_GAIN                 = 0x34;
    constexpr uint8_t CURRENT_P_GAIN                  = 0x36;
    constexpr uint8_t CURRENT_I_GAIN                  = 0x38;

    constexpr uint8_t TIME_MAX                        = 0x3A;
    constexpr uint8_t POSITION_CCW_MAX                = 0x3E;
    constexpr uint8_t POSITION_CW_MAX                 = 0x42;
    constexpr uint8_t VELOCITY_CCW_MAX                = 0x46;
    constexpr uint8_t VELOCITY_CW_MAX                 = 0x48;
    constexpr uint8_t CURRENT_CCW_MAX                 = 0x4A;
    constexpr uint8_t CURRENT_CW_MAX                  = 0x4C;
    constexpr uint8_t CURRENT_LIMIT                   = 0x4E;

    constexpr uint8_t MOTOR_ACCTIME                   = 0x50;
    constexpr uint8_t MOTOR_DECTIME                   = 0x52;
    constexpr uint8_t MOTOR_MOTION_MODE               = 0x54;
    constexpr uint8_t MOTOR_SUB_TARGET                = 0x55;
    constexpr uint8_t MOTOR_MAIN_TARGET               = 0x58;
    constexpr uint8_t MOTOR_DIRECTION                 = 0x5B;
    constexpr uint8_t MOTOR_MOTION_PROFILE_RUNMODE    = 0x5C;
    constexpr uint8_t MOTOR_ENABLE                    = 0x5D;
    constexpr uint8_t MOTOR_BRAKE                     = 0x5E;
    constexpr uint8_t MOTOR_EXTERNAL_BRAKE            = 0x5F;

    constexpr uint8_t MOTOR_TIME_NOW                  = 0x60;
    constexpr uint8_t MOTOR_TIME_LEFT_NOW             = 0x63;
    constexpr uint8_t MOTOR_POSITION_NOW              = 0x66;
    constexpr uint8_t MOTOR_POSITION_DIR              = 0x69;
    constexpr uint8_t MOTOR_VELOCITY_NOW              = 0x6A;
    constexpr uint8_t MOTOR_VELOCITY_DIR              = 0x6C;
    constexpr uint8_t MOTOR_CURRENT_NOW               = 0x6D;
    constexpr uint8_t MOTOR_CURRENT_DIR               = 0x6F;
    constexpr uint8_t MOTOR_ENCODER_NOW               = 0x70;
    constexpr uint8_t MOTOR_ENCODER_NOW_DIR           = 0x78;
    constexpr uint8_t MOTOR_HALLSENSOR_NOW            = 0x79;
    constexpr uint8_t MOTOR_ABSOLUTE_VALUE_NOW        = 0x7A;

    constexpr uint8_t FIRMWARE_VERSION                = 0xF8;
    constexpr uint8_t PROTOCOL_VERSION                = 0xFA;
    constexpr uint8_t SOFTWARE_VERSION                = 0xFC;
}