#include "MotorStatus.h"
#include "SerialPortConnector.h"

MotorStatus::MotorStatus(Controller *controller) : controller(controller) {
}

unsigned int MotorStatus::getTimeNow() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_TIME_NOW, 0x03, 0x03
    };
    std::vector<uint8_t> response = controller->request(request, "Get time now");
    if (response.size() > 2) {
        *controller->time_now = (static_cast<unsigned int>(response[0]) << 16)
        | (static_cast<unsigned int>(response[1]) << 8) | response[2];
    }
    return *controller->time_now;
}

unsigned int MotorStatus::getTimeLeft() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_TIME_LEFT_NOW, 0x03, 0x03
    };
    std::vector<uint8_t> response = controller->request(request, "Get time left");
    if (response.size() > 2) {
        *controller->time_left = (static_cast<unsigned int>(response[0]) << 16)
        | (static_cast<unsigned int>(response[1]) << 8) | response[2];
    }
    return *controller->time_left;
}

unsigned int MotorStatus::getPositionNow() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_POSITION_NOW, 0x03, 0x03
    };
    std::vector<uint8_t> response = controller->request(request, "Get position now");
    if (response.size() > 2) {
        *controller->position_now = (static_cast<unsigned int>(response[0]) << 16)
        | (static_cast<unsigned int>(response[1]) << 8) | response[2];
    }
    return *controller->position_now;
}

uint8_t MotorStatus::getPositionDir() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_POSITION_DIR, 0x03, 0x01
    };
    std::vector<uint8_t> response = controller->request(request, "Get position dir");
    if (!response.empty()) {
        *controller->position_dir = response[0];
    }
    return *controller->position_dir;
}

uint16_t MotorStatus::getVelocityNow() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_VELOCITY_NOW, 0x03, 0x02
    };
    std::vector<uint8_t> response = controller->request(request, "Get velocity now");
    if (response.size() > 1) {
        *controller->velocity_now = (static_cast<uint16_t>(response[0]) << 8) | response[1];
    }
    return *controller->velocity_now;
}

uint8_t MotorStatus::getVelocityDir() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_VELOCITY_DIR, 0x03, 0x01
    };
    std::vector<uint8_t> response = controller->request(request, "Get velocity dir");
    if (!response.empty()) {
        *controller->velocity_dir = response[0];
    }
    return *controller->velocity_dir;
}

uint16_t MotorStatus::getCurrentNow() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_CURRENT_NOW, 0x03, 0x02
    };
    std::vector<uint8_t> response = controller->request(request, "Get current now");
    if (response.size() > 1) {
        *controller->current_now = (static_cast<uint16_t>(response[0]) << 8) | response[1];
    }
    return *controller->current_now;
}

uint8_t MotorStatus::getCurrentDir() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_CURRENT_DIR, 0x03, 0x01
    };
    std::vector<uint8_t> response = controller->request(request, "Get current dir");
    if (!response.empty()) {
        *controller->current_dir = response[0];
    }
    return *controller->current_dir;
}

uint64_t MotorStatus::getEncoderNow() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_ENCODER_NOW, 0x03, 0x08
    };
    std::vector<uint8_t> response = controller->request(request, "Get encoder now");
    if (response.size() > 7) {
        *controller->encoder_now = (static_cast<uint64_t>(response[0]) << 56)
        | (static_cast<uint64_t>(response[1]) << 48)
        | (static_cast<uint64_t>(response[2]) << 40)
        | (static_cast<uint64_t>(response[3]) << 32)
        | (static_cast<uint64_t>(response[4]) << 24)
        | (static_cast<uint64_t>(response[5]) << 16)
        | (static_cast<uint64_t>(response[6]) << 8)
        | (static_cast<uint64_t>(response[7]));
    }
    return *controller->encoder_now;
}

uint8_t MotorStatus::getEncoderDir() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_ENCODER_DIR, 0x03, 0x01
    };
    std::vector<uint8_t> response = controller->request(request, "Get encoder dir");
    if (!response.empty()) {
        *controller->encoder_dir = response[0];
    }
    return *controller->encoder_dir;
}

uint8_t MotorStatus::getHallsensorNow() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_HALLSENSOR_NOW, 0x03, 0x01
    };
    std::vector<uint8_t> response = controller->request(request, "Get hallsensor now");
    if (!response.empty()) {
        *controller->hallsensor_now = response[0];
    }
    return *controller->hallsensor_now;
}

uint16_t MotorStatus::getAbsoluteValueNow() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_ABSOLUTE_VALUE_NOW, 0x03, 0x02
    };
    std::vector<uint8_t> response = controller->request(request, "Get absolute now");
    if (response.size() > 1) {
        *controller->absolute_value_now = (static_cast<uint16_t>(response[0]) << 8) | response[1];
    }
    return *controller->absolute_value_now;
}

float MotorStatus::getVoltageNow() {
    std::vector<uint8_t> request = {
        0x02, Address::VOLTAGE_NOW, 0x03, 0x02
    };
    std::vector<uint8_t> response = controller->request(request, "Get voltage now");
    if (response.size() > 1) {
        uint16_t value = (static_cast<uint16_t>(response[0]) << 8) | response[1];
        *controller->voltage_now = static_cast<uint16_t>(value * 10);
    }
    return *controller->voltage_now;
}

uint8_t MotorStatus::getTemperatureNow() {
    std::vector<uint8_t> request = {
        0x02, Address::TEMPERATURE_NOW, 0x03, 0x01
    };
    std::vector<uint8_t> response = controller->request(request, "Get temperature now");
    if (!response.empty()) {
        *controller->temperature_now = response[0];
    }
    return *controller->temperature_now;
}

/*
 * Description: 모터 상태 값을 한 번에 받는 함수
 * Return value: 순서대로 현재 시간, 남은 시간, 현재 위치, 위치 방향, 현재 속도, 속도 방향, 현재 전체 전류, 전류 방향, 현재 엔코더 위치, 엔코더 방향
 */
std::vector<unsigned int> MotorStatus::getMotorStatus() {
    std::vector<uint8_t> request = {
        0x02, Address::MOTOR_TIME_NOW, 0x03, 0x19
    };
    std::vector<unsigned int> result;

    std::vector<uint8_t> response = controller->request(request, "Get Motor status");
    if (response.size() > 18) {
        *controller->time_now = (static_cast<unsigned int>(response[0]) << 16)
        | (static_cast<unsigned int>(response[1]) << 8) | response[2];
        *controller->time_left = *controller->time_left = (static_cast<unsigned int>(response[3]) << 16)
        | (static_cast<unsigned int>(response[4]) << 8) | response[5];
        *controller->position_now = (static_cast<unsigned int>(response[6]) << 16)
        | (static_cast<unsigned int>(response[7]) << 8) | response[8];
        *controller->position_dir = response[9];
        *controller->velocity_now = (static_cast<uint16_t>(response[10]) << 8) | response[11];
        *controller->velocity_dir = response[12];
        *controller->current_now = (static_cast<uint16_t>(response[13]) << 8) | response[14];
        *controller->current_dir = response[15];
        *controller->encoder_now = (static_cast<uint64_t>(response[16]) << 56)
        | (static_cast<uint64_t>(response[17]) << 48)
        | (static_cast<uint64_t>(response[18]) << 40)
        | (static_cast<uint64_t>(response[19]) << 32)
        | (static_cast<uint64_t>(response[20]) << 24)
        | (static_cast<uint64_t>(response[21]) << 16)
        | (static_cast<uint64_t>(response[22]) << 8)
        | (static_cast<uint64_t>(response[23]));
        *controller->encoder_dir = response[24];
    }
    result.push_back(*controller->time_now);
    result.push_back(*controller->time_left);
    result.push_back(*controller->position_now);
    result.push_back(*controller->position_dir);
    result.push_back(*controller->velocity_now);
    result.push_back(*controller->velocity_dir);
    result.push_back(*controller->current_now);
    result.push_back(*controller->current_dir);
    result.push_back(*controller->encoder_now);
    result.push_back(*controller->encoder_dir);
    return result;
}

