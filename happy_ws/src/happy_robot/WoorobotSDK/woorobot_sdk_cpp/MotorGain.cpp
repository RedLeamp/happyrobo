#include "MotorGain.h"
#include "SerialPortConnector.h"

MotorGain::MotorGain(Controller *controller) : controller(controller) {
}

uint8_t MotorGain::getPositionTotalGain() {
    std::vector<uint8_t> request = makeGetPositionTotalGainCommand();
    std::vector<uint8_t> response = controller->request(request, "Get position total gain");
    if (!response.empty()) {
        *controller->position_total_gain = response[0];
    }
    return *controller->position_total_gain;
}

void MotorGain::setPositionTotalGain(uint8_t value) {
    std::vector<uint8_t> request = makeSetPositionTotalGainCommand(value);
    controller->request(request, "Set position total gain " + std::to_string(value));
    *controller->position_total_gain = value;
}

uint8_t MotorGain::getVelocityTotalGain() {
    std::vector<uint8_t> request = makeGetVelocityTotalGainCommand();
    std::vector<uint8_t> response = controller->request(request, "Get velocity total gain");
    if (!response.empty()) {
        *controller->velocity_total_gain = response[0];
    }
    return *controller->velocity_total_gain;
}

void MotorGain::setVelocityTotalGain(uint8_t value) {
    std::vector<uint8_t> request = makeSetVelocityTotalGainCommand(value);
    controller->request(request, "Set velocity total gain " + std::to_string(value));
    *controller->velocity_total_gain = value;
}

uint8_t MotorGain::getCurrentTotalGain() {
    std::vector<uint8_t> request = makeGetCurrentTotalGainCommand();
    std::vector<uint8_t> response = controller->request(request, "Get current total gain");
    if (!response.empty()) {
        *controller->current_total_gain = response[0];
    }
    return *controller->current_total_gain;
}

void MotorGain::setCurrentTotalGain(uint8_t value) {
    std::vector<uint8_t> request = makeSetCurrentTotalGainCommand(value);
    controller->request(request, "Set current total gain " + std::to_string(value));
    *controller->current_total_gain = value;
}

uint16_t MotorGain::getPositionPGain() {
    std::vector<uint8_t> request = makeGetPositionPGainCommand();
    std::vector<uint8_t> response = controller->request(request, "Get position p gain");
    if (response.size() > 1) {
        *controller->position_p_gain = (static_cast<uint16_t>(response[0]) << 8) | (static_cast<uint16_t>(response[1]));
    }
    return *controller->position_p_gain;
}

void MotorGain::setPositionPGain(uint16_t value) {
    std::vector<uint8_t> request = makeSetPositionPGainCommand(value);
    controller->request(request, "Set position p gain " + std::to_string(value));
    *controller->position_p_gain = value;
}

uint16_t MotorGain::getVelocityPGain() {
    std::vector<uint8_t> request = makeGetVelocityPGainCommand();
    std::vector<uint8_t> response = controller->request(request, "Get velocity p gain");
    if (response.size() > 1) {
        *controller->velocity_p_gain = (static_cast<uint16_t>(response[0]) << 8) | (static_cast<uint16_t>(response[1]));
    }
    return *controller->velocity_p_gain;
}

void MotorGain::setVelocityPGain(uint16_t value) {
    std::vector<uint8_t> request = makeSetVelocityPGainCommand(value);
    controller->request(request, "Set velocity p gain " + std::to_string(value));
    *controller->velocity_p_gain = value;
}

uint16_t MotorGain::getVelocityIGain() {
    std::vector<uint8_t> request = makeGetVelocityIGainCommand();
    std::vector<uint8_t> response = controller->request(request, "Get velocity i gain");
    if (response.size() > 1) {
        *controller->velocity_i_gain = (static_cast<uint16_t>(response[0]) << 8) | (static_cast<uint16_t>(response[1]));
    }
    return *controller->velocity_i_gain;
}

void MotorGain::setVelocityIGain(uint16_t value) {
    std::vector<uint8_t> request = makeSetVelocityIGainCommand(value);
    controller->request(request, "Set velocity i gain " + std::to_string(value));
    *controller->velocity_i_gain = value;
}

uint16_t MotorGain::getCurrentPGain() {
    std::vector<uint8_t> request = makeGetCurrentPGainCommand();
    std::vector<uint8_t> response = controller->request(request, "Get current p gain");
    if (response.size() > 1) {
        *controller->current_p_gain = (static_cast<uint16_t>(response[0]) << 8) | (static_cast<uint16_t>(response[1]));
    }
    return *controller->current_p_gain;
}

void MotorGain::setCurrentPGain(uint16_t value) {
    std::vector<uint8_t> request = makeSetCurrentPGainCommand(value);
    controller->request(request, "Set current p gain " + std::to_string(value));
    *controller->current_p_gain = value;
}

uint16_t MotorGain::getCurrentIGain() {
    std::vector<uint8_t> request = makeGetCurrentIGainCommand();
    std::vector<uint8_t> response = controller->request(request, "Get current i gain");
    if (response.size() > 1) {
        *controller->current_i_gain = (static_cast<uint16_t>(response[0]) << 8) | (static_cast<uint16_t>(response[1]));
    }
    return *controller->current_i_gain;
}

void MotorGain::setCurrentIGain(uint16_t value) {
    std::vector<uint8_t> request = makeSetCurrentIGainCommand(value);
    controller->request(request, "Set current i gain " + std::to_string(value));
    *controller->current_i_gain = value;
}


std::vector<uint8_t> MotorGain::makeGetPositionTotalGainCommand() {
    std::vector<uint8_t> request = { 0x02, Address::POSITION_TOTAL_GAIN, 0x03, 0x01 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorGain::makeSetPositionTotalGainCommand(uint8_t value) {
    std::vector<uint8_t> request = { 0x01, Address::POSITION_TOTAL_GAIN, 0x03, value };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorGain::makeGetVelocityTotalGainCommand() {
    std::vector<uint8_t> request = { 0x02, Address::VELOCITY_TOTAL_GAIN, 0x03, 0x01 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorGain::makeSetVelocityTotalGainCommand(uint8_t value) {
    std::vector<uint8_t> request = { 0x01, Address::VELOCITY_TOTAL_GAIN, 0x03, value };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorGain::makeGetCurrentTotalGainCommand() {
    std::vector<uint8_t> request = { 0x02, Address::CURRENT_TOTAL_GAIN, 0x03, 0x01 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorGain::makeSetCurrentTotalGainCommand(uint8_t value) {
    std::vector<uint8_t> request = { 0x01, Address::CURRENT_TOTAL_GAIN, 0x03, value };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorGain::makeGetPositionPGainCommand() {
    std::vector<uint8_t> request = { 0x02, Address::POSITION_P_GAIN, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorGain::makeSetPositionPGainCommand(uint16_t value) {
    uint8_t high = value >> 8;
    uint8_t low = value & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::POSITION_P_GAIN, 0x04, high, low };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorGain::makeGetVelocityPGainCommand() {
    std::vector<uint8_t> request = { 0x02, Address::VELOCITY_P_GAIN, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorGain::makeSetVelocityPGainCommand(uint16_t value) {
    uint8_t high = value >> 8;
    uint8_t low = value & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::VELOCITY_P_GAIN, 0x04, high, low };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorGain::makeGetVelocityIGainCommand() {
    std::vector<uint8_t> request = { 0x02, Address::VELOCITY_I_GAIN, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorGain::makeSetVelocityIGainCommand(uint16_t value) {
    uint8_t high = value >> 8;
    uint8_t low = value & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::VELOCITY_I_GAIN, 0x04, high, low };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorGain::makeGetCurrentPGainCommand() {
    std::vector<uint8_t> request = { 0x02, Address::CURRENT_P_GAIN, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorGain::makeSetCurrentPGainCommand(uint16_t value) {
    uint8_t high = value >> 8;
    uint8_t low = value & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::CURRENT_P_GAIN, 0x04, high, low };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorGain::makeGetCurrentIGainCommand() {
    std::vector<uint8_t> request = { 0x02, Address::CURRENT_I_GAIN, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorGain::makeSetCurrentIGainCommand(uint16_t value) {
    uint8_t high = value >> 8;
    uint8_t low = value & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::CURRENT_I_GAIN, 0x04, high, low };
    return controller->makeRequest(request);
}

void MotorGain::setMotorGain(uint8_t positionTotalGain, uint8_t velocityTotalGain, uint8_t currentTotalGain,
    uint16_t positionPGain, uint16_t velocityPGain, uint16_t velocityIGain, uint16_t currentPGain,
    uint16_t currentIGain) {
    setPositionTotalGain(positionTotalGain);
    setVelocityTotalGain(velocityTotalGain);
    setCurrentTotalGain(currentTotalGain);
    setPositionPGain(positionPGain);
    setVelocityPGain(velocityPGain);
    setVelocityIGain(velocityIGain);
    setCurrentPGain(currentPGain);
    setCurrentIGain(currentIGain);
}

void MotorGain::init() {
    getPositionTotalGain();
    getVelocityTotalGain();
    getCurrentTotalGain();
    getPositionPGain();
    getVelocityPGain();
    getVelocityIGain();
    getCurrentPGain();
    getCurrentIGain();
}
