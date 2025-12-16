#pragma once

#include <cstdint>
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>

class Controller;

class Firmware {
public:
    Firmware(Controller* controller);
    void firmwareUpdate(const char* file_path);

private:
    Controller* controller;
    std::vector<uint8_t> requestBootLoader(std::vector<uint8_t> request);
    uint8_t hexCharToByte(char c);
    uint8_t hexPairToByte(char high, char low);
};
