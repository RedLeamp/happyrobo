#include "MotorLimit.h"

#include "SerialPortConnector.h"

MotorLimit::MotorLimit(Controller *controller) : controller(controller) {
}

unsigned int MotorLimit::getTimeMax() {
    std::vector<uint8_t> request = makeGetTimeMaxCommand();
    std::vector<uint8_t> response = controller->request(request, "Get time max");
    if (response.size() > 3) {
        *controller->time_max = (static_cast<unsigned int>(response[0]) << 24) |
                                (static_cast<unsigned int>(response[1]) << 16) |
                                (static_cast<unsigned int>(response[2]) << 8) | response[3];
    }
    return *controller->time_max;
}

void MotorLimit::setTimeMax(unsigned int value) {
    std::vector<uint8_t> request = makeSetTimeMaxCommand(value);
    controller->request(request, "Set time max " + std::to_string(value));
    *controller->time_max = value;
}

double MotorLimit::getPositionCcwMax() {
    std::vector<uint8_t> request = makeGetPositionCcwMaxCommand();
    std::vector<uint8_t> response = controller->request(request, "Get position ccw max");
    if (response.size() > 3) {
        unsigned long value = (static_cast<unsigned int>(response[0]) << 24) |
                              (static_cast<unsigned int>(response[1]) << 16) |
                              (static_cast<unsigned int>(response[2]) << 8) |
                              response[3];
        *controller->position_ccw_max = static_cast<double>(value) * 0.1;
    }
    return *controller->position_ccw_max;
}

void MotorLimit::setPositionCcwMax(double value) {
    std::vector<uint8_t> request = makeSetPositionCcwMaxCommand(value);
    controller->request(request, "Set position ccw max " + std::to_string(value));
    *controller->position_ccw_max = static_cast<double>(static_cast<unsigned int>(value * 10)) * 0.1;
}

double MotorLimit::getPositionCwMax() {
    std::vector<uint8_t> request = makeGetPositionCwMaxCommand();
    std::vector<uint8_t> response = controller->request(request, "Get position cw max");
    if (response.size() > 3) {
        unsigned long value = (static_cast<unsigned int>(response[0]) << 24) |
                              (static_cast<unsigned int>(response[1]) << 16) |
                              (static_cast<unsigned int>(response[2]) << 8) | response[3];
        *controller->position_cw_max = static_cast<double>(value) * 0.1;
    }
    return *controller->position_cw_max;
}

void MotorLimit::setPositionCwMax(double value) {
    std::vector<uint8_t> request = makeSetPositionCwMaxCommand(value);
    controller->request(request, "Set position cw max " + std::to_string(value));
    *controller->position_cw_max = static_cast<double>(static_cast<unsigned int>(value * 10)) * 0.1;
}

float MotorLimit::getVelocityCcwMax() {
    std::vector<uint8_t> request = makeGetVelocityCcwMaxCommand();
    std::vector<uint8_t> response = controller->request(request, "Get velocity ccw max");
    if (response.size() > 1) {
        uint16_t value = (static_cast<uint16_t>(response[0]) << 8) | response[1];
        *controller->velocity_ccw_max = static_cast<float>(value) * 0.1;
    }
    return *controller->velocity_ccw_max;
}

void MotorLimit::setVelocityCcwMax(float value) {
    std::vector<uint8_t> request = makeSetVelocityCcwMaxCommand(value);
    controller->request(request, "Set velocity ccw max " + std::to_string(value));
    *controller->velocity_ccw_max = static_cast<float>(static_cast<uint16_t>(value * 10)) * 0.1;
}

float MotorLimit::getVelocityCwMax() {
    std::vector<uint8_t> request = makeGetVelocityCwMaxCommand();
    std::vector<uint8_t> response = controller->request(request, "Get velocity cw max");
    if (response.size() > 1) {
        uint16_t value = (static_cast<uint16_t>(response[0]) << 8) | response[1];
        *controller->velocity_cw_max = static_cast<float>(value) * 0.1;
    }
    return *controller->velocity_cw_max;
}

void MotorLimit::setVelocityCwMax(float value) {
    std::vector<uint8_t> request = makeSetVelocityCwMaxCommand(value);
    controller->request(request, "Set velocity cw max " + std::to_string(value));
    *controller->velocity_cw_max = static_cast<float>(static_cast<uint16_t>(value * 10)) * 0.1;
}

float MotorLimit::getCurrentCcwMax() {
    std::vector<uint8_t> request = makeGetCurrentCcwMaxCommand();
    std::vector<uint8_t> response = controller->request(request, "Get current ccw max");
    if (response.size() > 1) {
        uint16_t value = (static_cast<uint16_t>(response[0]) << 8) | response[1];
        *controller->current_ccw_max = static_cast<float>(value) * 0.1;
    }
    return *controller->current_ccw_max;
}

void MotorLimit::setCurrentCcwMax(float value) {
    std::vector<uint8_t> request = makeSetCurrentCcwMaxCommand(value);
    controller->request(request, "Set current ccw max " + std::to_string(value));
    *controller->current_ccw_max = static_cast<float>(static_cast<uint16_t>(value * 10)) * 0.1;
}

float MotorLimit::getCurrentCwMax() {
    std::vector<uint8_t> request = makeGetCurrentCwMaxCommand();
    std::vector<uint8_t> response = controller->request(request, "Get current cw max");
    if (response.size() > 1) {
        uint16_t value = (static_cast<uint16_t>(response[0]) << 8) | response[1];
        *controller->current_cw_max = static_cast<float>(value) * 0.1;
    }
    return *controller->current_cw_max;
}

void MotorLimit::setCurrentCwMax(float value) {
    std::vector<uint8_t> request = makeSetCurrentCwMaxCommand(value);
    controller->request(request, "Set current cw max " + std::to_string(value));
    *controller->current_cw_max = static_cast<float>(static_cast<uint16_t>(value * 10)) * 0.1;
}

float MotorLimit::getCurrentLimit() {
    std::vector<uint8_t> request = makeGetCurrentLimitCommand();
    std::vector<uint8_t> response = controller->request(request, "Get current limit");
    if (response.size() > 1) {
        uint16_t value = (static_cast<uint16_t>(response[0]) << 8) | response[1];
        *controller->current_limit = static_cast<float>(value) * 0.1;
    }
    return *controller->current_limit;
}

void MotorLimit::setCurrentLimit(float value) {
    std::vector<uint8_t> request = makeSetCurrentLimitCommand(value);
    controller->request(request, "Set current limit " + std::to_string(value));
    *controller->current_limit = static_cast<float>(static_cast<uint16_t>(value * 10)) * 0.1;
}

std::vector<uint8_t> MotorLimit::makeGetTimeMaxCommand() {
    std::vector<uint8_t> request = { 0x02, Address::TIME_MAX, 0x03, 0x04 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorLimit::makeSetTimeMaxCommand(unsigned int value) {
    uint8_t b1 = (value >> 24) & 0xFF;
    uint8_t b2 = (value >> 16) & 0xFF;
    uint8_t b3 = (value >> 8) & 0xFF;
    uint8_t b4 = value & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::TIME_MAX, 0x06, b1, b2, b3, b4 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorLimit::makeGetPositionCcwMaxCommand() {
    std::vector<uint8_t> request = { 0x02, Address::POSITION_CCW_MAX, 0x03, 0x04 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorLimit::makeSetPositionCcwMaxCommand(double value) {
    unsigned long intValue = static_cast<unsigned int>(value * 10);
    uint8_t b1 = (intValue >> 24) & 0xFF;
    uint8_t b2 = (intValue >> 16) & 0xFF;
    uint8_t b3 = (intValue >> 8) & 0xFF;
    uint8_t b4 = intValue & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::POSITION_CCW_MAX, 0x06, b1, b2, b3, b4 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorLimit::makeGetPositionCwMaxCommand() {
    std::vector<uint8_t> request = { 0x02, Address::POSITION_CW_MAX, 0x03, 0x04 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorLimit::makeSetPositionCwMaxCommand(double value) {
    unsigned long intValue = static_cast<unsigned int>(value * 10);
    uint8_t b1 = (intValue >> 24) & 0xFF;
    uint8_t b2 = (intValue >> 16) & 0xFF;
    uint8_t b3 = (intValue >> 8) & 0xFF;
    uint8_t b4 = intValue & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::POSITION_CW_MAX, 0x06, b1, b2, b3, b4 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorLimit::makeGetVelocityCcwMaxCommand() {
    std::vector<uint8_t> request = { 0x02, Address::VELOCITY_CCW_MAX, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorLimit::makeSetVelocityCcwMaxCommand(float value) {
    uint16_t intValue = static_cast<uint16_t>(value * 10);
    uint8_t high = (intValue >> 8) & 0xFF;
    uint8_t low = intValue & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::VELOCITY_CCW_MAX, 0x04, high, low };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorLimit::makeGetVelocityCwMaxCommand() {
    std::vector<uint8_t> request = { 0x02, Address::VELOCITY_CW_MAX, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorLimit::makeSetVelocityCwMaxCommand(float value) {
    uint16_t intValue = static_cast<uint16_t>(value * 10);
    uint8_t high = (intValue >> 8) & 0xFF;
    uint8_t low = intValue & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::VELOCITY_CW_MAX, 0x04, high, low };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorLimit::makeGetCurrentCcwMaxCommand() {
    std::vector<uint8_t> request = { 0x02, Address::CURRENT_CCW_MAX, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorLimit::makeSetCurrentCcwMaxCommand(float value) {
    uint16_t intValue = static_cast<uint16_t>(value * 10);
    uint8_t high = (intValue >> 8) & 0xFF;
    uint8_t low = intValue & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::CURRENT_CCW_MAX, 0x04, high, low };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorLimit::makeGetCurrentCwMaxCommand() {
    std::vector<uint8_t> request = { 0x02, Address::CURRENT_CW_MAX, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorLimit::makeSetCurrentCwMaxCommand(float value) {
    uint16_t intValue = static_cast<uint16_t>(value * 10);
    uint8_t high = (intValue >> 8) & 0xFF;
    uint8_t low = intValue & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::CURRENT_CW_MAX, 0x04, high, low };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorLimit::makeGetCurrentLimitCommand() {
    std::vector<uint8_t> request = { 0x02, Address::CURRENT_LIMIT, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorLimit::makeSetCurrentLimitCommand(float value) {
    uint16_t intValue = static_cast<uint16_t>(value * 10);
    uint8_t high = (intValue >> 8) & 0xFF;
    uint8_t low = intValue & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::CURRENT_LIMIT, 0x04, high, low };
    return controller->makeRequest(request);
}

void MotorLimit::setMotorLimit(unsigned int timeMax, double positionCcwMax, double positionCwMax, float velocityCcwMax,
    float velocityCwMax, float currentCcwMax, float currentCwMax, float currentLimit) {
    setTimeMax(timeMax);
    setPositionCcwMax(positionCcwMax);
    setPositionCwMax(positionCwMax);
    setVelocityCcwMax(velocityCcwMax);
    setVelocityCwMax(velocityCwMax);
    setCurrentCcwMax(currentCcwMax);
    setCurrentCwMax(currentCwMax);
    setCurrentLimit(currentLimit);
}

void MotorLimit::init() {
    getTimeMax();
    getPositionCcwMax();
    getPositionCwMax();
    getVelocityCcwMax();
    getVelocityCwMax();
    getCurrentCcwMax();
    getCurrentCwMax();
    getCurrentLimit();
}
