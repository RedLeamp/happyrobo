#pragma once

#include <vector>
#include <cstdint>

class Controller;

class MotorStatus {
public:
    MotorStatus(Controller* controller);

    unsigned int getTimeNow();
    unsigned int getTimeLeft();
    unsigned int getPositionNow();
    uint8_t getPositionDir();
    uint16_t getVelocityNow();
    uint8_t getVelocityDir();
    uint16_t getCurrentNow();
    uint8_t getCurrentDir();
    uint64_t getEncoderNow();
    uint8_t getEncoderDir();
    uint8_t getHallsensorNow();
    uint16_t getAbsoluteValueNow();
    float getVoltageNow();
    uint8_t getTemperatureNow();

    std::vector<unsigned int> getMotorStatus();

private:
    Controller* controller;
};
