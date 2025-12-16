#pragma once

#include <vector>
#include <cstdint>

class Controller;

class MotorParam {
public:
    MotorParam(Controller *controller);

    uint8_t getDInductanceConstant();

    void setDInductanceConstant(uint8_t value);

    uint8_t getQInductanceConstant();

    void setQInductanceConstant(uint8_t value);

    uint8_t getFluxLinkageConstant();

    void setFluxLinkageConstant(uint8_t value);

    uint16_t getResistanceConstant();

    void setResistanceConstant(uint16_t value);

    uint16_t getInductanceConstant();

    void setInductanceConstant(uint16_t value);

    uint16_t getTorqueConstant();

    void setTorqueConstant(uint16_t value);

    uint16_t getMomentOfInertia();

    void setMomentOfInertia(uint16_t value);

    float getRatedSpeed();

    void setRatedSpeed(float value);

    float getRatedCurrent();

    void setRatedCurrent(float value);

    float getRatedVoltage();

    void setRatedVoltage(float value);

    std::vector<uint8_t> makeGetDInductanceConstantCommand();
    std::vector<uint8_t> makeSetDInductanceConstantCommand(uint8_t value);

    std::vector<uint8_t> makeGetQInductanceConstantCommand();
    std::vector<uint8_t> makeSetQInductanceConstantCommand(uint8_t value);

    std::vector<uint8_t> makeGetFluxLinkageConstantCommand();
    std::vector<uint8_t> makeSetFluxLinkageConstantCommand(uint8_t value);

    std::vector<uint8_t> makeGetResistanceConstantCommand();
    std::vector<uint8_t> makeSetResistanceConstantCommand(uint16_t value);

    std::vector<uint8_t> makeGetInductanceConstantCommand();
    std::vector<uint8_t> makeSetInductanceConstantCommand(uint16_t value);

    std::vector<uint8_t> makeGetTorqueConstantCommand();
    std::vector<uint8_t> makeSetTorqueConstantCommand(uint16_t value);

    std::vector<uint8_t> makeGetMomentOfInertiaCommand();
    std::vector<uint8_t> makeSetMomentOfInertiaCommand(uint16_t value);

    std::vector<uint8_t> makeGetRatedSpeedCommand();
    std::vector<uint8_t> makeSetRatedSpeedCommand(float value);

    std::vector<uint8_t> makeGetRatedCurrentCommand();
    std::vector<uint8_t> makeSetRatedCurrentCommand(float value);

    std::vector<uint8_t> makeGetRatedVoltageCommand();
    std::vector<uint8_t> makeSetRatedVoltageCommand(float value);

    void setMotorParam(uint8_t dInductanceConstant, uint8_t qInductanceConstant, uint8_t fluxLinkageConstant,
                       uint16_t resistanceConstant, uint16_t inductanceConstant, uint16_t torqueConstant,
                       uint16_t momentOfInertia, float ratedSpeed, float ratedCurrent, float ratedVoltage);

    void init();

private:
    Controller *controller;
};

