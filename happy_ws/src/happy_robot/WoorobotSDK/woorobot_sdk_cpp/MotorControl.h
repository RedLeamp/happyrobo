#pragma once

#include <vector>
#include <cstdint>

class Controller;

class MotorControl {
public:
    MotorControl(Controller *controller);

    uint16_t getAccTime();

    void setAccTime(uint16_t value);

    uint16_t getDecTime();

    void setDecTime(uint16_t value);

    uint8_t getMotionMode();

    void setMotionMode(uint8_t value);

    unsigned int getSubTarget();

    void setSubTarget(unsigned int value);

    unsigned int getMainTarget();

    void setMainTarget(unsigned int value);

    uint8_t getMotorDirection();

    void setMotorDirection(uint8_t value);

    uint8_t getRunMode();

    void setRunMode(uint8_t value);

    uint8_t getMotorEnable();

    void setMotorEnable(uint8_t value);

    uint8_t getMotorBrake();

    void setMotorBrake(uint8_t value);

    uint8_t getExternalBrake();

    void setExternalBrake(uint8_t value);

    std::vector<uint8_t> makeGetAccTimeCommand();
    std::vector<uint8_t> makeSetAccTimeCommand(uint16_t value);
    std::vector<uint8_t> makeGetDecTimeCommand();
    std::vector<uint8_t> makeSetDecTimeCommand(uint16_t value);
    std::vector<uint8_t> makeGetMotionModeCommand();
    std::vector<uint8_t> makeSetMotionModeCommand(uint8_t value);
    std::vector<uint8_t> makeGetSubTargetCommand();
    std::vector<uint8_t> makeSetSubTargetCommand(unsigned int value);
    std::vector<uint8_t> makeGetMainTargetCommand();
    std::vector<uint8_t> makeSetMainTargetCommand(unsigned int value);
    std::vector<uint8_t> makeGetMotorDirectionCommand();
    std::vector<uint8_t> makeSetMotorDirectionCommand(uint8_t value);
    std::vector<uint8_t> makeGetRunModeCommand();
    std::vector<uint8_t> makeSetRunModeCommand(uint8_t value);
    std::vector<uint8_t> makeGetMotorEnableCommand();
    std::vector<uint8_t> makeSetMotorEnableCommand(uint8_t value);
    std::vector<uint8_t> makeGetMotorBrakeCommand();
    std::vector<uint8_t> makeSetMotorBrakeCommand(uint8_t value);
    std::vector<uint8_t> makeGetExternalBrakeCommand();
    std::vector<uint8_t> makeSetExternalBrakeCommand(uint8_t value);

    void setMotorControl(uint16_t accTime, uint16_t decTime, uint8_t motionMode, unsigned int subTarget,
                    unsigned int mainTarget, uint8_t motorDirection, uint8_t runMode, uint8_t motorBrake,
                    uint8_t externalBrake);

    void init();

private:
    Controller *controller;
};
