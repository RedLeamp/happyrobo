#include "MotorParam.h"
#include "SerialPortConnector.h"

MotorParam::MotorParam(Controller *controller) : controller(controller) {
}

uint8_t MotorParam::getDInductanceConstant() {
    std::vector<uint8_t> request = makeGetDInductanceConstantCommand();
    std::vector<uint8_t> response = controller->request(request, "Get D inductance constant");
    if (!response.empty()) {
        *controller->d_inductance_constant = response[0];
    }
    return *controller->d_inductance_constant;
}

void MotorParam::setDInductanceConstant(uint8_t value) {
    std::vector<uint8_t> request = makeSetDInductanceConstantCommand(value);
    controller->request(request, "Set D inductance constant " + std::to_string(value));
    *controller->d_inductance_constant = value;
}

uint8_t MotorParam::getQInductanceConstant() {
    std::vector<uint8_t> request = makeGetQInductanceConstantCommand();
    std::vector<uint8_t> response = controller->request(request, "Get Q inductance constant");
    if (!response.empty()) {
        *controller->q_inductance_constant = response[0];
    }
    return *controller->q_inductance_constant;
}

void MotorParam::setQInductanceConstant(uint8_t value) {
    std::vector<uint8_t> request = makeSetQInductanceConstantCommand(value);
    controller->request(request, "Set Q inductance constant " + std::to_string(value));
    *controller->q_inductance_constant = value;
}

uint8_t MotorParam::getFluxLinkageConstant() {
    std::vector<uint8_t> request = makeGetFluxLinkageConstantCommand();
    std::vector<uint8_t> response = controller->request(request, "Get flux linkage constant");
    if (!response.empty()) {
        *controller->flux_linkage_constant = response[0];
    }
    return *controller->flux_linkage_constant;
}

void MotorParam::setFluxLinkageConstant(uint8_t value) {
    std::vector<uint8_t> request = makeSetFluxLinkageConstantCommand(value);
    controller->request(request, "Set flux linkage constant " + std::to_string(value));
    *controller->flux_linkage_constant = value;
}

uint16_t MotorParam::getResistanceConstant() {
    std::vector<uint8_t> request = makeGetResistanceConstantCommand();
    std::vector<uint8_t> response = controller->request(request, "Get resistance constant");
    if (response.size() > 1) {
        *controller->resistance_constant = (static_cast<uint16_t>(response[0]) << 8) | response[1];
    }
    return *controller->resistance_constant;
}

void MotorParam::setResistanceConstant(uint16_t value) {
    std::vector<uint8_t> request = makeSetResistanceConstantCommand(value);
    controller->request(request, "Set resistance constant " + std::to_string(value));
    *controller->resistance_constant = value;
}

uint16_t MotorParam::getInductanceConstant() {
    std::vector<uint8_t> request = makeGetInductanceConstantCommand();
    std::vector<uint8_t> response = controller->request(request, "Get inductance constant");
    if (response.size() > 1) {
        *controller->inductance_constant = (static_cast<uint16_t>(response[0]) << 8) | response[1];
    }
    return *controller->inductance_constant;
}

void MotorParam::setInductanceConstant(uint16_t value) {
    std::vector<uint8_t> request = makeSetInductanceConstantCommand(value);
    controller->request(request, "Set inductance constant " + std::to_string(value));
    *controller->inductance_constant = value;
}

uint16_t MotorParam::getTorqueConstant() {
    std::vector<uint8_t> request = makeGetTorqueConstantCommand();
    std::vector<uint8_t> response = controller->request(request, "Get torque constant");
    if (response.size() > 1) {
        *controller->torque_constant = (static_cast<uint16_t>(response[0]) << 8) | response[1];
    }
    return *controller->torque_constant;
}

void MotorParam::setTorqueConstant(uint16_t value) {
    std::vector<uint8_t> request = makeSetTorqueConstantCommand(value);
    controller->request(request, "Set torque constant " + std::to_string(value));
    *controller->torque_constant = value;
}

uint16_t MotorParam::getMomentOfInertia() {
    std::vector<uint8_t> request = makeGetMomentOfInertiaCommand();
    std::vector<uint8_t> response = controller->request(request, "Get moment of inertia");
    if (response.size() > 1) {
        *controller->moment_of_inertia = (static_cast<uint16_t>(response[0]) << 8) | response[1];
    }
    return *controller->moment_of_inertia;
}

void MotorParam::setMomentOfInertia(uint16_t value) {
    std::vector<uint8_t> request = makeSetMomentOfInertiaCommand(value);
    controller->request(request, "Set moment of inertia " + std::to_string(value));
    *controller->moment_of_inertia = value;
}

float MotorParam::getRatedSpeed() {
    std::vector<uint8_t> request = makeGetRatedSpeedCommand();
    std::vector<uint8_t> response = controller->request(request, "Get rated speed");
    if (response.size() > 1) {
        uint16_t value = (static_cast<uint16_t>(response[0]) << 8) | response[1];
        *controller->rated_speed = static_cast<float>(value) * 0.1f;
    }
    return *controller->rated_speed;
}

void MotorParam::setRatedSpeed(float value) {
    std::vector<uint8_t> request = makeSetRatedSpeedCommand(value);
    controller->request(request, "Set rated speed " + std::to_string(value));
    *controller->rated_speed = static_cast<float>(static_cast<uint16_t>(value * 10)) * 0.1f;
}

float MotorParam::getRatedCurrent() {
    std::vector<uint8_t> request = makeGetRatedCurrentCommand();
    std::vector<uint8_t> response = controller->request(request, "Get rated current");
    if (response.size() > 1) {
        uint16_t value = (static_cast<uint16_t>(response[0]) << 8) | response[1];
        *controller->rated_current = static_cast<float>(value) * 0.1f;
    }
    return *controller->rated_current;
}

void MotorParam::setRatedCurrent(float value) {
    std::vector<uint8_t> request = makeSetRatedCurrentCommand(value);
    controller->request(request, "Set rated current " + std::to_string(value));
    *controller->rated_current = static_cast<float>(static_cast<uint16_t>(value * 10)) * 0.1f;
}

float MotorParam::getRatedVoltage() {
    std::vector<uint8_t> request = makeGetRatedVoltageCommand();
    std::vector<uint8_t> response = controller->request(request, "Get rated voltage");
    if (!response.empty()) {
        *controller->rated_voltage = static_cast<float>(response[0]) * 0.1f;
    }
    return *controller->rated_voltage;
}

void MotorParam::setRatedVoltage(float value) {
    std::vector<uint8_t> request = makeSetRatedVoltageCommand(value);
    controller->request(request, "Set rated voltage " + std::to_string(value));
    *controller->rated_voltage = static_cast<float>(static_cast<uint8_t>(value * 10)) * 0.1f;
}


std::vector<uint8_t> MotorParam::makeGetDInductanceConstantCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_D_INDUCTANCE_CONSTANT, 0x03, 0x01 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeSetDInductanceConstantCommand(uint8_t value) {
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_D_INDUCTANCE_CONSTANT, 0x03, value };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeGetQInductanceConstantCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_Q_INDUCTANCE_CONSTANT, 0x03, 0x01 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeSetQInductanceConstantCommand(uint8_t value) {
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_Q_INDUCTANCE_CONSTANT, 0x03, value };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeGetFluxLinkageConstantCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_FLUX_LINKAGE_CONSTANT, 0x03, 0x01 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeSetFluxLinkageConstantCommand(uint8_t value) {
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_FLUX_LINKAGE_CONSTANT, 0x03, value };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeGetResistanceConstantCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_RESISTANCE_CONSTANT, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeSetResistanceConstantCommand(uint16_t value) {
    uint8_t high = value >> 8;
    uint8_t low = value & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_RESISTANCE_CONSTANT, 0x04, high, low };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeGetInductanceConstantCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_INDUCTANCE_CONSTANT, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeSetInductanceConstantCommand(uint16_t value) {
    uint8_t high = value >> 8;
    uint8_t low = value & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_INDUCTANCE_CONSTANT, 0x04, high, low };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeGetTorqueConstantCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_TORQUE_CONSTANT, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeSetTorqueConstantCommand(uint16_t value) {
    uint8_t high = value >> 8;
    uint8_t low = value & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_TORQUE_CONSTANT, 0x04, high, low };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeGetMomentOfInertiaCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_MOMENT_OF_INERTIA_CONSTANT, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeSetMomentOfInertiaCommand(uint16_t value) {
    uint8_t high = value >> 8;
    uint8_t low = value & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_MOMENT_OF_INERTIA_CONSTANT, 0x04, high, low };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeGetRatedSpeedCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_RATED_SPEED, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeSetRatedSpeedCommand(float value) {
    uint16_t intValue = static_cast<uint16_t>(value * 10);
    uint8_t high = intValue >> 8;
    uint8_t low = intValue & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_RATED_SPEED, 0x04, high, low };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeGetRatedCurrentCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_RATED_CURRENT, 0x03, 0x02 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeSetRatedCurrentCommand(float value) {
    uint16_t intValue = static_cast<uint16_t>(value * 10);
    uint8_t high = intValue >> 8;
    uint8_t low = intValue & 0xFF;
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_RATED_CURRENT, 0x04, high, low };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeGetRatedVoltageCommand() {
    std::vector<uint8_t> request = { 0x02, Address::MOTOR_RATED_VOLTAGE, 0x03, 0x01 };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MotorParam::makeSetRatedVoltageCommand(float value) {
    uint8_t intValue = static_cast<uint8_t>(value * 10);
    std::vector<uint8_t> request = { 0x01, Address::MOTOR_RATED_VOLTAGE, 0x03, intValue };
    return controller->makeRequest(request);
}


void MotorParam::setMotorParam(uint8_t dInductanceConstant, uint8_t qInductanceConstant, uint8_t fluxLinkageConstant,
    uint16_t resistanceConstant, uint16_t inductanceConstant, uint16_t torqueConstant, uint16_t momentOfInertia,
    float ratedSpeed, float ratedCurrent, float ratedVoltage) {
    setDInductanceConstant(dInductanceConstant);
    setQInductanceConstant(qInductanceConstant);
    setFluxLinkageConstant(fluxLinkageConstant);
    setResistanceConstant(resistanceConstant);
    setInductanceConstant(inductanceConstant);
    setTorqueConstant(torqueConstant);
    setMomentOfInertia(momentOfInertia);
    setRatedSpeed(ratedSpeed);
    setRatedCurrent(ratedCurrent);
    setRatedVoltage(ratedVoltage);
}

void MotorParam::init() {
    getDInductanceConstant();
    getQInductanceConstant();
    getFluxLinkageConstant();
    getResistanceConstant();
    getInductanceConstant();
    getTorqueConstant();
    getMomentOfInertia();
    getRatedSpeed();
    getRatedCurrent();
    getRatedVoltage();
}


