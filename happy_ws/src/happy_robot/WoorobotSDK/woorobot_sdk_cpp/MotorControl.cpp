#include "MotorControl.h"
#include "SerialPortConnector.h"

MotorControl::MotorControl(Controller *controller) : controller(controller) {
}

uint16_t MotorControl::getAccTime() {
    std::vector<uint8_t> request = makeGetAccTimeCommand();
    std::vector<uint8_t> response = controller->request(request, "Get acc time");
    if (response.size() > 1) {
        *controller->acc_time = (static_cast<uint16_t>(response[0]) << 8) | response[1];
    }
    return *controller->acc_time;
}

void MotorControl::setAccTime(uint16_t value) {
    std::vector<uint8_t> request = makeSetAccTimeCommand(value);
    controller->requestWithoutResponse(request, "Set acc time " + std::to_string(value));
    *controller->acc_time = value;
}

uint16_t MotorControl::getDecTime() {
    std::vector<uint8_t> request = makeGetDecTimeCommand();
    std::vector<uint8_t> response = controller->request(request, "Get dec time");
    if (response.size() > 1) {
        *controller->dec_time = (static_cast<uint16_t>(response[0]) << 8) | response[1];
    }
    return *controller->dec_time;
}

void MotorControl::setDecTime(uint16_t value) {
    std::vector<uint8_t> request = makeSetDecTimeCommand(value);
    controller->requestWithoutResponse(request, "Set dec time " + std::to_string(value));
    *controller->dec_time = value;
}

uint8_t MotorControl::getMotionMode() {
    std::vector<uint8_t> request = makeGetMotionModeCommand();
    std::vector<uint8_t> response = controller->request(request, "Get motion mode");
    if (!response.empty()) {
        *controller->motion_mode = response[0];
    }
    return *controller->motion_mode;
}

void MotorControl::setMotionMode(uint8_t value) {
    std::vector<uint8_t> request = makeSetMotionModeCommand(value);
    controller->requestWithoutResponse(request, "Set motion mode " + std::to_string(value));
    *controller->motion_mode = value;
}

unsigned int MotorControl::getSubTarget() {
    std::vector<uint8_t> request = makeGetSubTargetCommand();
    std::vector<uint8_t> response = controller->request(request, "Get sub target");
    if (response.size() > 2) {
        *controller->sub_target = (static_cast<unsigned int>(response[0]) << 16) |
                                 (static_cast<unsigned int>(response[1]) << 8) |
                                 (static_cast<unsigned int>(response[1]));
    }
    return *controller->sub_target;
}

void MotorControl::setSubTarget(unsigned int value) {
    std::vector<uint8_t> request = makeSetSubTargetCommand(value);
    controller->requestWithoutResponse(request, "Set sub target " + std::to_string(value));
    *controller->sub_target = value;
}

unsigned int MotorControl::getMainTarget() {
    std::vector<uint8_t> request = makeGetMainTargetCommand();
    std::vector<uint8_t> response = controller->request(request, "Get main target");
    if (response.size() > 2) {
        *controller->main_target = (static_cast<unsigned int>(response[0]) << 16) |
                                  (static_cast<unsigned int>(response[1]) << 8) |
                                  (static_cast<unsigned int>(response[1]));
    }
    return *controller->main_target;
}

void MotorControl::setMainTarget(unsigned int value) {
    std::vector<uint8_t> request = makeSetMainTargetCommand(value);

    std::cout << "\n(setMainTarget TXD) ";
    for (int i = 0; i < request.size(); i++) {
        uint8_t byte = request[i];
        std::cout << "0x" << std::hex << std::uppercase << std::setw(2)
                << std::setfill('0') << static_cast<int>(byte);
        if (i < request.size() - 1) std::cout << " ";
    }

    controller->requestWithoutResponse(request, "Set main target " + std::to_string(value));
    *controller->main_target = value;
}

uint8_t MotorControl::getMotorDirection() {
    std::vector<uint8_t> request = makeGetMotorDirectionCommand();
    std::vector<uint8_t> response = controller->request(request, "Get motor direction");
    if (!response.empty()) {
        *controller->motor_direction = response[0];
    }
    return *controller->motor_direction;
}

void MotorControl::setMotorDirection(uint8_t value) {
    std::vector<uint8_t> request = makeSetMotorDirectionCommand(value);

    std::cout << "\n(setMotorDirection TXD) ";
    for (int i = 0; i < request.size(); i++) {
        uint8_t byte = request[i];
        std::cout << "0x" << std::hex << std::uppercase << std::setw(2)
                << std::setfill('0') << static_cast<int>(byte);
        if (i < request.size() - 1) std::cout << " ";
    }

    controller->requestWithoutResponse(request, "Set motor direction " + std::to_string(value));
    *controller->motor_direction = value;
}

uint8_t MotorControl::getRunMode() {
    std::vector<uint8_t> request = makeGetRunModeCommand();
    std::vector<uint8_t> response = controller->request(request, "Get run mode");
    if (!response.empty()) {
        *controller->run_mode = response[0];
    }
    return *controller->run_mode;
}

void MotorControl::setRunMode(uint8_t value) {
    std::vector<uint8_t> request = makeSetRunModeCommand(value);
    controller->requestWithoutResponse(request, "Set run mode " + std::to_string(value));
    *controller->run_mode = value;
}

uint8_t MotorControl::getMotorEnable() {
    std::vector<uint8_t> request = makeGetMotorEnableCommand();
    std::vector<uint8_t> response = controller->request(request, "Get motor enable");
    if (!response.empty()) {
        *controller->motor_enable = response[0];
    }
    return *controller->motor_enable;
}

void MotorControl::setMotorEnable(uint8_t value) {
    std::vector<uint8_t> request = makeSetMotorEnableCommand(value);
    controller->requestWithoutResponse(request, "Set motor enable " + std::to_string(value));
    *controller->motor_enable = value;
}

uint8_t MotorControl::getMotorBrake() {
    std::vector<uint8_t> request = makeGetMotorBrakeCommand();
    std::vector<uint8_t> response = controller->request(request, "Get motor brake");
    if (!response.empty()) {
        *controller->motor_brake = response[0];
    }
    return *controller->motor_brake;
}

void MotorControl::setMotorBrake(uint8_t value) {
    std::vector<uint8_t> request = makeSetMotorBrakeCommand(value);
    controller->requestWithoutResponse(request, "Set motor brake " + std::to_string(value));
    *controller->motor_brake = value;
}

uint8_t MotorControl::getExternalBrake() {
    std::vector<uint8_t> request = makeGetExternalBrakeCommand();
    std::vector<uint8_t> response = controller->request(request, "Get external brake");
    if (!response.empty()) {
        *controller->external_brake = response[0];
    }
    return *controller->external_brake;
}

void MotorControl::setExternalBrake(uint8_t value) {
    std::vector<uint8_t> request = makeSetExternalBrakeCommand(value);
    controller->requestWithoutResponse(request, "Set external brake " + std::to_string(value));
    *controller->external_brake = value;
}

std::vector<uint8_t> MotorControl::makeGetAccTimeCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_ACCTIME, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeSetAccTimeCommand(uint16_t value) {
    uint8_t high = value >> 8;
    uint8_t low = value & 0xff;
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_ACCTIME, 0x04, high, low };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeGetDecTimeCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_DECTIME, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeSetDecTimeCommand(uint16_t value) {
    uint8_t high = value >> 8;
    uint8_t low = value & 0xff;
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_DECTIME, 0x04, high, low };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeGetMotionModeCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_MOTION_MODE, 0x03, 0x01 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeSetMotionModeCommand(uint8_t value) {
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_MOTION_MODE, 0x03, value };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeGetSubTargetCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_SUB_TARGET, 0x03, 0x03 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeSetSubTargetCommand(unsigned int value) {
    uint8_t b1 = (value >> 16) & 0xFF;
    uint8_t b2 = (value >> 8) & 0xFF;
    uint8_t b3 = value & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_SUB_TARGET, 0x05, b1, b2, b3 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeGetMainTargetCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_MAIN_TARGET, 0x03, 0x03 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeSetMainTargetCommand(unsigned int value) {
    uint8_t b1 = (value >> 16) & 0xFF;
    uint8_t b2 = (value >> 8) & 0xFF;
    uint8_t b3 = value & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_MAIN_TARGET, 0x05, b1, b2, b3 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeGetMotorDirectionCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_DIRECTION, 0x03, 0x01 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeSetMotorDirectionCommand(uint8_t value) {
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_DIRECTION, 0x03, value };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeGetRunModeCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_MOTION_PROFILE_RUNMODE, 0x03, 0x01 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeSetRunModeCommand(uint8_t value) {
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_MOTION_PROFILE_RUNMODE, 0x03, value };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeGetMotorEnableCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_ENABLE, 0x03, 0x01 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeSetMotorEnableCommand(uint8_t value) {
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_ENABLE, 0x03, value };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeGetMotorBrakeCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_BRAKE, 0x03, 0x01 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeSetMotorBrakeCommand(uint8_t value) {
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_BRAKE, 0x03, value };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeGetExternalBrakeCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_EXTERNAL_BRAKE, 0x03, 0x01 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorControl::makeSetExternalBrakeCommand(uint8_t value) {
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_EXTERNAL_BRAKE, 0x03, value };
    return controller->makeRequest(request);
}


void MotorControl::setMotorControl(uint16_t accTime, uint16_t decTime, uint8_t motionMode, unsigned int subTarget,
    unsigned int mainTarget, uint8_t motorDirection, uint8_t runMode, uint8_t motorBrake, uint8_t externalBrake) {
    setAccTime(accTime);
    setDecTime(decTime);
    setMotionMode(motionMode);
    setSubTarget(subTarget);
    setMainTarget(mainTarget);
    setMotorDirection(motorDirection);
    setRunMode(runMode);
    setMotorBrake(motorBrake);
    setExternalBrake(externalBrake);
}

void MotorControl::init() {
    getAccTime();
    getDecTime();
    getMotionMode();
    getMainTarget();
    getSubTarget();
    getRunMode();
}

