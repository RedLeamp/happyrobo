#pragma once

#include <vector>
#include <cstdint>

class Controller;

class MotorConfig {
public:
    MotorConfig(Controller *controller);

    uint8_t getMotorType();

    void setMotorType(uint8_t value);

    uint8_t getMotorEncoderDirection();

    void setMotorEncoderDirection(uint8_t value);

    uint16_t getMotorEncoderCpr();

    void setMotorEncoderCpr(uint16_t value);

    uint16_t getElectronicAngle();

    void setElectronicAngle(uint16_t value);

    float getGearRatio();

    void setGearRatio(float value);

    uint8_t getLeadAngleMode();

    void setLeadAngleMode(uint8_t value);

    uint8_t getForceAngle();

    void setForceAngle(uint8_t value);

    uint16_t getElectricalZero();

    void setElectricalZero(uint16_t value);

    uint8_t getHallSensorSet();

    void setHallSensorSet(uint8_t value);

    std::vector<uint8_t> makeGetMotorTypeCommand();
    std::vector<uint8_t> makeSetMotorTypeCommand(uint8_t value);

    std::vector<uint8_t> makeGetMotorEncoderDirectionCommand();
    std::vector<uint8_t> makeSetMotorEncoderDirectionCommand(uint8_t value);

    std::vector<uint8_t> makeGetMotorEncoderCprCommand();
    std::vector<uint8_t> makeSetMotorEncoderCprCommand(uint16_t value);

    std::vector<uint8_t> makeGetElectronicAngleCommand();
    std::vector<uint8_t> makeSetElectronicAngleCommand(uint16_t value);

    std::vector<uint8_t> makeGetGearRatioCommand();
    std::vector<uint8_t> makeSetGearRatioCommand(float value);

    std::vector<uint8_t> makeGetLeadAngleModeCommand();
    std::vector<uint8_t> makeSetLeadAngleModeCommand(uint8_t value);

    std::vector<uint8_t> makeGetForceAngleCommand();
    std::vector<uint8_t> makeSetForceAngleCommand(uint8_t value);

    std::vector<uint8_t> makeGetElectricalZeroCommand();
    std::vector<uint8_t> makeSetElectricalZeroCommand(uint16_t value);

    std::vector<uint8_t> makeGetHallSensorSetCommand();
    std::vector<uint8_t> makeSetHallSensorSetCommand(uint8_t value);

    void setMotorConfig(uint8_t motorType, uint8_t encoderDir, uint16_t encoderCpr, uint16_t electronicAngle,
                        float gearRatio, uint8_t leadAngleMode, uint8_t forceAngle, uint16_t electricalZero,
                        uint8_t hallSensor);

    void init();

private:
    Controller *controller;
};
