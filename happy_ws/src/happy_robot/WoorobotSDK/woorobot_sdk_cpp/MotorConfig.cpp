#include "MotorConfig.h"
#include "SerialPortConnector.h"

MotorConfig::MotorConfig(Controller* controller) : controller(controller) {
}

void MotorConfig::init() {
    getMotorType();
    getMotorEncoderDirection();
    getMotorEncoderCpr();
    getElectronicAngle();
    getGearRatio();
    getLeadAngleMode();
    getForceAngle();
    getElectricalZero();
    getHallSensorSet();
}

uint8_t MotorConfig::getMotorType() {
    std::vector<uint8_t> request = makeGetMotorTypeCommand();
    std::vector<uint8_t> response = controller->request(request, "Get motor type");
    if (!response.empty()) {
        *controller->motor_type = response[0];
    }
    return *controller->motor_type;
}

void MotorConfig::setMotorType(uint8_t value) {
    std::vector<uint8_t> request = makeSetMotorTypeCommand(value);
    controller->request(request, "Set motor type " + std::to_string(value));
    *controller->motor_type = value;
}

uint8_t MotorConfig::getMotorEncoderDirection() {
    std::vector<uint8_t> request = makeGetMotorEncoderDirectionCommand();
    std::vector<uint8_t> response = controller->request(request, "Get motor encoder direction");
    if (!response.empty()) {
        *controller->motor_encoder_direction = static_cast<int>(response[0]);
    }
    return *controller->motor_encoder_direction;
}

void MotorConfig::setMotorEncoderDirection(uint8_t value) {
    std::vector<uint8_t> request = makeSetMotorEncoderDirectionCommand(value);
    controller->request(request, "Set motor encoder direction " + std::to_string(value));
    *controller->motor_encoder_direction = value;
}

uint16_t MotorConfig::getMotorEncoderCpr() {
    std::vector<uint8_t> request = makeGetMotorEncoderCprCommand();
    std::vector<uint8_t> response = controller->request(request, "Get motor encoder speed");
    if (response.size() > 1) {
        *controller->motor_encoder_cpr = (static_cast<int>(response[0]) << 8) | (static_cast<int>(response[1]));
    }
    return *controller->motor_encoder_cpr;
}

void MotorConfig::setMotorEncoderCpr(uint16_t value) {
    std::vector<uint8_t> request = makeSetMotorEncoderCprCommand(value);

    controller->request(request, "Set motor encoder speed " + std::to_string(value));
    *controller->motor_encoder_cpr = value;
}

uint16_t MotorConfig::getElectronicAngle() {
    std::vector<uint8_t> request = makeGetElectronicAngleCommand();
    std::vector<uint8_t> response = controller->request(request, "Get electronic angle");
    if (response.size() > 1) {
        *controller->electronic_angle = (static_cast<uint16_t>(response[0]) << 8) | (static_cast<uint16_t>(response[1]));
    }
    return *controller->electronic_angle;
}

void MotorConfig::setElectronicAngle(uint16_t value) {
    std::vector<uint8_t> request = makeSetElectronicAngleCommand(value);
    controller->request(request, "Set electronic angle " + std::to_string(value));
    *controller->electronic_angle = value;
}

float MotorConfig::getGearRatio() {
    std::vector<uint8_t> request = makeGetGearRatioCommand();
    std::vector<uint8_t> response = controller->request(request, "Get gear ratio");
    if (response.size() > 1) {
        uint16_t value = (static_cast<uint16_t>(response[0]) << 8) | (static_cast<uint16_t>(response[1]));
        *controller->gear_ratio = static_cast<float>(value) * 0.1;
    }
    return *controller->gear_ratio;
}

void MotorConfig::setGearRatio(float value) {
    uint16_t intValue = static_cast<uint16_t>(value * 10);
    std::vector<uint8_t> request = makeSetGearRatioCommand(value);

    controller->request(request, "Set gear ratio " + std::to_string(value));
    *controller->gear_ratio = static_cast<float>(intValue) * 0.1;
}

uint8_t MotorConfig::getLeadAngleMode() {
    std::vector<uint8_t> request = makeGetLeadAngleModeCommand();
    std::vector<uint8_t> response = controller->request(request, "Get lead angle mode");
    if (response.size() > 0) {
        *controller->lead_angle_mode = response[0];
    }
    return *controller->lead_angle_mode;
}

void MotorConfig::setLeadAngleMode(uint8_t value) {
    std::vector<uint8_t> request = makeSetLeadAngleModeCommand(value);
    controller->request(request, "Set lead angle mode " + std::to_string(value));
    *controller->lead_angle_mode = value;
}

uint8_t MotorConfig::getForceAngle() {
    std::vector<uint8_t> request = makeGetForceAngleCommand();
    std::vector<uint8_t> response = controller->request(request, "Get force angle");
    if (response.size() > 0) {
        *controller->force_angle = response[0];
    }
    return *controller->force_angle;
}

void MotorConfig::setForceAngle(uint8_t value) {
    std::vector<uint8_t> request = makeSetForceAngleCommand(value);
    controller->request(request, "Set force angle " + std::to_string(value));
    *controller->force_angle = value;
}

uint16_t MotorConfig::getElectricalZero() {
    std::vector<uint8_t> request = makeGetElectricalZeroCommand();
    std::vector<uint8_t> response = controller->request(request, "Get electrical zero");
    if (response.size() > 1) {
        *controller->electrical_zero = (static_cast<uint16_t>(response[0]) << 8) | (static_cast<uint16_t>(response[1]));
    }
    return *controller->electrical_zero;
}

void MotorConfig::setElectricalZero(uint16_t value) {
    std::vector<uint8_t> request = makeSetElectricalZeroCommand(value);
    controller->request(request, "Set electrical zero " + std::to_string(value));
    *controller->electrical_zero = value;
}

uint8_t MotorConfig::getHallSensorSet() {
    std::vector<uint8_t> request = makeGetHallSensorSetCommand();
    std::vector<uint8_t> response = controller->request(request, "Get hall sensor set");
    if (response.size() > 0) {
        *controller->hallsensor_set = response[0];
    }
    return *controller->hallsensor_set;
}

void MotorConfig::setHallSensorSet(uint8_t value) {
    std::vector<uint8_t> request = makeSetHallSensorSetCommand(value);
    controller->request(request, "Set hall sensor setting " + std::to_string(value));
    *controller->hallsensor_set = value;
}

std::vector<uint8_t> MotorConfig::makeGetMotorTypeCommand() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_TYPE, 0x03, 0x01
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorConfig::makeSetMotorTypeCommand(uint8_t value) {
    std::vector<uint8_t> request = {
        0x01, Address::MOTOR_TYPE, 0x03, value
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorConfig::makeGetMotorEncoderDirectionCommand() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_ENCODER_DIR, 0x03, 0x01
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorConfig::makeSetMotorEncoderDirectionCommand(uint8_t value) {
    std::vector<uint8_t> request = {
        0x01, Address::MOTOR_ENCODER_DIR, 0x03, value
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorConfig::makeGetMotorEncoderCprCommand() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_ENCODER_CPR, 0x03, 0x02
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorConfig::makeSetMotorEncoderCprCommand(uint16_t value) {
    uint8_t high = value >> 8;
    uint8_t low = value & 0xFF;
    std::vector<uint8_t> request = {
        0x01, Address::MOTOR_ENCODER_CPR, 0x04, high, low
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorConfig::makeGetElectronicAngleCommand() {
    std::vector<uint8_t> request = {
        0x02, Address::ELECTRONIC_ANGLE, 0x03, 0x02
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorConfig::makeSetElectronicAngleCommand(uint16_t value) {
    uint8_t high = value >> 8;
    uint8_t low = value & 0xFF;
    std::vector<uint8_t> request = {
        0x01, Address::ELECTRONIC_ANGLE, 0x04, high, low
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorConfig::makeGetGearRatioCommand() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_GEAR, 0x03, 0x02
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorConfig::makeSetGearRatioCommand(float value) {
    uint16_t intValue = static_cast<uint16_t>(value * 10);
    uint8_t high = intValue >> 8;
    uint8_t low = intValue & 0xFF;
    std::vector<uint8_t> request = {
        0x01, Address::MOTOR_GEAR, 0x04, high, low
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorConfig::makeGetLeadAngleModeCommand() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_LEAD_ANGLE_MODE, 0x03, 0x01
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorConfig::makeSetLeadAngleModeCommand(uint8_t value) {
    std::vector<uint8_t> request = {
        0x01, Address::MOTOR_LEAD_ANGLE_MODE, 0x03, value
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorConfig::makeGetForceAngleCommand() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_FORCE_ANGLE_LEVEL, 0x03, 0x01
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorConfig::makeSetForceAngleCommand(uint8_t value) {
    std::vector<uint8_t> request = {
        0x01, Address::MOTOR_FORCE_ANGLE_LEVEL, 0x03, value
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorConfig::makeGetElectricalZeroCommand() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_ELECTRICAL_ZERO, 0x03, 0x02
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorConfig::makeSetElectricalZeroCommand(uint16_t value) {
    uint8_t high = value >> 8;
    uint8_t low = value & 0xFF;
    std::vector<uint8_t> request = {
        0x01, Address::MOTOR_ELECTRICAL_ZERO, 0x04, high, low
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorConfig::makeGetHallSensorSetCommand() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_HALLSENSOR_SETTING, 0x03, 0x01
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorConfig::makeSetHallSensorSetCommand(uint8_t value) {
    std::vector<uint8_t> request = {
        0x01, Address::MOTOR_HALLSENSOR_SETTING, 0x03, value
    };
    return controller->makeRequest(request);
}

void MotorConfig::setMotorConfig(uint8_t motorType, uint8_t encoderDir, uint16_t encoderCpr, uint16_t electronicAngle,
                                 float gearRatio, uint8_t leadAngleMode, uint8_t forceAngle, uint16_t electricalZero, uint8_t hallSensor) {
    setMotorType(motorType);
    setMotorEncoderDirection(encoderDir);
    setMotorEncoderCpr(encoderCpr);
    setElectronicAngle(electronicAngle);
    setGearRatio(gearRatio);
    setLeadAngleMode(leadAngleMode);
    setForceAngle(forceAngle);
    setElectricalZero(electricalZero);
    setHallSensorSet(hallSensor);
}

