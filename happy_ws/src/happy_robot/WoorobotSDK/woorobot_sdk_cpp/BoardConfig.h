#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <iostream>

class Controller;

class BoardConfig {
public:
    BoardConfig(Controller* controller);

    void softwareReset();
    void locationReset();
    void factoryReset();
    void setId(uint8_t id);
    void setError(uint8_t value);
    uint8_t getError();
    void setMaxVoltage(float value);
    float getMaxVoltage();
    void setMinVoltage(float value);
    float getMinVoltage();
    void setMaxTemperature(uint8_t value);
    uint8_t getMaxTemperature();

    std::vector<uint8_t> makeSoftwareResetCommand();
    std::vector<uint8_t> makeLocationResetCommand();
    std::vector<uint8_t> makeFactoryResetCommand();
    std::vector<uint8_t> makeSetIdCommand(uint8_t id);
    std::vector<uint8_t> makeSetErrorCommand(uint8_t value);
    std::vector<uint8_t> makeGetErrorCommand();
    std::vector<uint8_t> makeSetMaxVoltageCommand(float value);
    std::vector<uint8_t> makeGetMaxVoltageCommand();
    std::vector<uint8_t> makeSetMinVoltageCommand(float value);
    std::vector<uint8_t> makeGetMinVoltageCommand();
    std::vector<uint8_t> makeSetMaxTemperatureCommand(uint8_t value);
    std::vector<uint8_t> makeGetMaxTemperatureCommand();

    void setBoardConfig(uint8_t id, uint8_t error, float maxVoltage, float minVoltage, float maxTemperature);

    void init();

private:
    Controller* controller;
};