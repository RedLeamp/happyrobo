#pragma once

#include <cstdint>
#include <vector>
#include <string>

class Controller;

class MainCommConfig {
public:
    MainCommConfig(Controller* controller);

    uint8_t getCommMode();
    void setCommMode(uint8_t value);
    uint8_t getWatchdog();
    void setWatchdog(uint8_t value);

    std::vector<uint8_t> makeGetCommModeCommand();
    std::vector<uint8_t> makeSetCommModeCommand(uint8_t value);
    std::vector<uint8_t> makeGetWatchdogCommand();
    std::vector<uint8_t> makeSetWatchdogCommand(uint8_t value);

    void setMainCommConfig(uint8_t commMode, uint8_t watchdog);

    void init();

private:
    Controller* controller;
};