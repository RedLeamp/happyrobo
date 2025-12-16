#pragma once

#include <vector>
#include <cstdint>

class Controller;

class MotorGain {
public:
    MotorGain(Controller* controller);

    uint8_t getPositionTotalGain();
    void setPositionTotalGain(uint8_t value);
    uint8_t getVelocityTotalGain();
    void setVelocityTotalGain(uint8_t value);
    uint8_t getCurrentTotalGain();
    void setCurrentTotalGain(uint8_t value);
    uint16_t getPositionPGain();
    void setPositionPGain(uint16_t value);
    uint16_t getVelocityPGain();
    void setVelocityPGain(uint16_t value);
    uint16_t getVelocityIGain();
    void setVelocityIGain(uint16_t value);
    uint16_t getCurrentPGain();
    void setCurrentPGain(uint16_t value);
    uint16_t getCurrentIGain();
    void setCurrentIGain(uint16_t value);

    std::vector<uint8_t> makeGetPositionTotalGainCommand();
    std::vector<uint8_t> makeSetPositionTotalGainCommand(uint8_t value);

    std::vector<uint8_t> makeGetVelocityTotalGainCommand();
    std::vector<uint8_t> makeSetVelocityTotalGainCommand(uint8_t value);

    std::vector<uint8_t> makeGetCurrentTotalGainCommand();
    std::vector<uint8_t> makeSetCurrentTotalGainCommand(uint8_t value);

    std::vector<uint8_t> makeGetPositionPGainCommand();
    std::vector<uint8_t> makeSetPositionPGainCommand(uint16_t value);

    std::vector<uint8_t> makeGetVelocityPGainCommand();
    std::vector<uint8_t> makeSetVelocityPGainCommand(uint16_t value);

    std::vector<uint8_t> makeGetVelocityIGainCommand();
    std::vector<uint8_t> makeSetVelocityIGainCommand(uint16_t value);

    std::vector<uint8_t> makeGetCurrentPGainCommand();
    std::vector<uint8_t> makeSetCurrentPGainCommand(uint16_t value);

    std::vector<uint8_t> makeGetCurrentIGainCommand();
    std::vector<uint8_t> makeSetCurrentIGainCommand(uint16_t value);

    void setMotorGain(uint8_t positionTotalGain, uint8_t velocityTotalGain, uint8_t currentTotalGain,
        uint16_t positionPGain, uint16_t velocityPGain, uint16_t velocityIGain, uint16_t currentPGain,
        uint16_t currentIGain);

    void init();

private:
    Controller* controller;
};
