#pragma once

#include <vector>
#include <cstdint>

class Controller;

class MotorLimit {
public:
    MotorLimit(Controller* controller);

    unsigned int getTimeMax();
    void setTimeMax(unsigned int value);
    double getPositionCcwMax();
    void setPositionCcwMax(double value);
    double getPositionCwMax();
    void setPositionCwMax(double value);
    float getVelocityCcwMax();
    void setVelocityCcwMax(float value);
    float getVelocityCwMax();
    void setVelocityCwMax(float value);
    float getCurrentCcwMax();
    void setCurrentCcwMax(float value);
    float getCurrentCwMax();
    void setCurrentCwMax(float value);
    float getCurrentLimit();
    void setCurrentLimit(float value);

    std::vector<uint8_t> makeGetTimeMaxCommand();
    std::vector<uint8_t> makeSetTimeMaxCommand(unsigned int value);

    std::vector<uint8_t> makeGetPositionCcwMaxCommand();
    std::vector<uint8_t> makeSetPositionCcwMaxCommand(double value);

    std::vector<uint8_t> makeGetPositionCwMaxCommand();
    std::vector<uint8_t> makeSetPositionCwMaxCommand(double value);

    std::vector<uint8_t> makeGetVelocityCcwMaxCommand();
    std::vector<uint8_t> makeSetVelocityCcwMaxCommand(float value);

    std::vector<uint8_t> makeGetVelocityCwMaxCommand();
    std::vector<uint8_t> makeSetVelocityCwMaxCommand(float value);

    std::vector<uint8_t> makeGetCurrentCcwMaxCommand();
    std::vector<uint8_t> makeSetCurrentCcwMaxCommand(float value);

    std::vector<uint8_t> makeGetCurrentCwMaxCommand();
    std::vector<uint8_t> makeSetCurrentCwMaxCommand(float value);

    std::vector<uint8_t> makeGetCurrentLimitCommand();
    std::vector<uint8_t> makeSetCurrentLimitCommand(float value);

    void setMotorLimit(unsigned int timeMax, double positionCcwMax, double positionCwMax,
        float velocityCcwMax, float velocityCwMax, float currentCcwMax, float currentCwMax, float currentLimit);

    void init();

private:
    Controller* controller;
};
