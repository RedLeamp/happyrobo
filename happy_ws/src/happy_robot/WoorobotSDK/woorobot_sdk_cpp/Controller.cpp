#include "../include/Controller.h"
#include "SerialPortConnector.h"

Controller::Controller(uint8_t id, SerialPortConnector *serial_port_connector) : connector(serial_port_connector),
    boardConfig(this), mainCommConfig(this), motorConfig(this),
    motorParam(this), motorGain(this), motorLimit(this),
    motorControl(this), motorStatus(this), firmware(this) {
    init();
    *selectedId = id;
}


void Controller::delete_ptr() const {
    delete selectedId;
    delete error;
    delete max_voltage;
    delete min_voltage;
    delete max_temperature;
    delete comm_mode;
    delete watchdog;
    delete motor_type;
    delete motor_encoder_direction;
    delete motor_encoder_cpr;
    delete electronic_angle;
    delete gear_ratio;
    delete lead_angle_mode;
    delete force_angle;
    delete electrical_zero;
    delete hallsensor_set;
    delete d_inductance_constant;
    delete q_inductance_constant;
    delete flux_linkage_constant;
    delete resistance_constant;
    delete inductance_constant;
    delete torque_constant;
    delete moment_of_inertia;
    delete rated_speed;
    delete rated_current;
    delete rated_voltage;
    delete position_total_gain;
    delete velocity_total_gain;
    delete current_total_gain;
    delete position_p_gain;
    delete velocity_p_gain;
    delete velocity_i_gain;
    delete current_p_gain;
    delete current_i_gain;
    delete time_max;
    delete position_ccw_max;
    delete position_cw_max;
    delete velocity_ccw_max;
    delete velocity_cw_max;
    delete current_ccw_max;
    delete current_cw_max;
    delete current_limit;
    delete acc_time;
    delete dec_time;
    delete motion_mode;
    delete sub_target;
    delete main_target;
    delete motor_direction;
    delete run_mode;
    delete motor_enable;
    delete motor_brake;
    delete external_brake;
    delete time_now;
    delete time_left;
    delete position_now;
    delete position_dir;
    delete velocity_now;
    delete velocity_dir;
    delete current_now;
    delete current_dir;
    delete encoder_now;
    delete encoder_dir;
    delete hallsensor_now;
    delete absolute_value_now;
    delete voltage_now;
    delete temperature_now;
}

void Controller::init() {
    selectedId = new uint8_t(0x00);
    error = new uint8_t(0x01);
    max_voltage = new float(0x01);
    min_voltage = new float(0x01);
    max_temperature = new uint8_t(0x01);

    comm_mode = new uint8_t(0x01);
    watchdog = new uint8_t(0x00);

    motor_type = new uint8_t(0x00);
    motor_encoder_direction = new uint8_t(0x01);
    motor_encoder_cpr = new uint16_t(0x00);
    electronic_angle = new uint16_t(0x00);
    gear_ratio = new float(0x01);
    lead_angle_mode = new uint8_t(0x00);
    force_angle = new uint8_t(0x00);
    electrical_zero = new uint16_t(3600);
    hallsensor_set = new uint8_t(0x01);

    d_inductance_constant = new uint8_t();
    q_inductance_constant = new uint8_t();
    flux_linkage_constant = new uint8_t();
    resistance_constant = new uint16_t(0x01);
    inductance_constant = new uint16_t(0x01);
    torque_constant = new uint16_t(0x01);
    moment_of_inertia = new uint16_t(0x01);
    rated_speed = new float(0x01);
    rated_current = new float(0x01);
    rated_voltage = new float();

    position_total_gain = new uint8_t();
    velocity_total_gain = new uint8_t();
    current_total_gain = new uint8_t();
    position_p_gain = new uint16_t(0x01);
    velocity_p_gain = new uint16_t(0x01);
    velocity_i_gain = new uint16_t(0x01);
    current_p_gain = new uint16_t(0x01);
    current_i_gain = new uint16_t(0x01);

    time_max = new unsigned int(0x01);
    position_ccw_max = new double(0x01);
    position_cw_max = new double(0x01);
    velocity_ccw_max = new float(0x01);
    velocity_cw_max = new float(0x01);
    current_ccw_max = new float(0x01);
    current_cw_max = new float(0x01);
    current_limit = new float(0x01);

    acc_time = new uint16_t();
    dec_time = new uint16_t();
    motion_mode = new uint8_t();
    sub_target = new unsigned int();
    main_target = new unsigned int();
    motor_direction = new uint8_t();
    run_mode = new uint8_t();
    motor_enable = new uint8_t();
    motor_brake = new uint8_t();
    external_brake = new uint8_t();

    time_now = new unsigned int();
    time_left = new unsigned int();
    position_now = new unsigned int();
    position_dir = new uint8_t();
    velocity_now = new uint16_t();
    velocity_dir = new uint8_t();
    current_now = new uint16_t();
    current_dir = new uint8_t();
    encoder_now = new uint64_t();
    encoder_dir = new uint8_t();
    hallsensor_now = new uint8_t();
    absolute_value_now = new uint16_t();
    voltage_now = new float(0x00);
    temperature_now = new uint8_t(0x00);
}

/*
 * Description: 현재 시간을 로그용 string으로 반환하는 함수
 * Return value: 현재 시각이 2025년 06월 27일 13시 36분 23.273241초 라면 "2025-06-27 13:36:23:273241" 로 리턴
 */
std::string Controller::getCurrentTimeString() {
    using namespace std::chrono;

    auto now = system_clock::now();
    auto now_time_t = system_clock::to_time_t(now);
    auto microsec = duration_cast<microseconds>(now.time_since_epoch()) % 1'000'000;
    std::tm *tm = std::localtime(&now_time_t);

    std::ostringstream oss;
    oss << std::put_time(tm, "%Y-%m-%d %H:%M:%S")
            << ":" << std::setw(6) << std::setfill('0') << microsec.count();
    return oss.str();
}

std::string Controller::get_port_path() {
#if defined(__linux__)
    char path[64];
    snprintf(path, sizeof(path), "/proc/self/fd/%d", *connector->fd);
    char resolved[PATH_MAX];
    ssize_t len = readlink(path, resolved, sizeof(resolved) - 1);
    if (len != -1) {
        resolved[len] = '\0';
        return std::string(resolved);
    }
#endif
    return "";
}

/*
 * Description: request의 CRC를 계산하고 제어기에 데이터 전송 후 응답 데이터 리턴
 * Parameter: request - 명령어 (CRC 및 접두사 id 까지 제외한 데이터 또는 전체 명령어 데이터)
 *            description - 로그에 남길 설명
 * Return value : 제어기의 응답이 있는 경우 값 부분만 리턴, 예외 발생 시 에러 로그 후 빈 값 리턴
 */
std::vector<uint8_t> Controller::request(std::vector<uint8_t> request, std::string description) {
    if (*connector->baudrate < 0) {
        // std::cerr << "Cannot request without baudrate value" << std::endl;
        return {};
    }
    if (connector->selectedPort->length() < 1) {
        // std::cerr << "Cannot request without port" << std::endl;
        return {};
    }

    std::vector<uint8_t> request_crc = request;

    if (request.size() > 3 && request[0] != 0xFF) {
        request_crc = makeRequest(request);
    }

    if (connector->fd == nullptr || *connector->fd < 0 || get_port_path() != *connector->selectedPort) {
        if (*connector->fd > 0) {
            close(*connector->fd);
        }
        connector->connectPort(*connector->selectedPort, *connector->baudrate);
    }
    if (*connector->fd < 0) {
        // std::cerr << "Failed to open port" << std::endl;
        return {};
    }

    write(*connector->fd, request_crc.data(), request_crc.size());

    std::cout << getCurrentTimeString() << " (TXD) " << description << "(";
    for (int i = 0; i < request_crc.size(); i++) {
        uint8_t byte = request_crc[i];
        std::cout << "0x" << std::hex << std::uppercase << std::setw(2)
                << std::setfill('0') << static_cast<int>(byte);
        if (i < request_crc.size() - 1) std::cout << " ";
    }
    std::cout << ")" << std::endl;

    std::vector<uint8_t> response;
    char buf[64];
    int totalTimeout = 10;
    int interByteTimeout = 5;
    std::vector<uint8_t> result_c;

    auto now_ms = [this]() {
        timeval tv;
        getTimevalNow(&tv);
        return tv.tv_sec * 1000 + tv.tv_usec / 1000;
    };

    if (request_crc[4] == 0x02) {
        int start_time = now_ms();
        int last_read_time = start_time;
        int count = 0;
        int length = 0;
        int valueCnt = 0;

        while (true) {
            int n = 0;
            n = read(*connector->fd, buf, sizeof(buf));
            if (n > 0) {
                response.insert(response.end(), buf, buf + n);
                last_read_time = now_ms();

                // 응답 길이 추적
                for (int i = 0; i < n; ++i) {
                    count++;
                    size_t idx = response.size() - n + i;

                    switch (count) {
                        case 1:
                        case 2: if (response[idx] != 0xFF) {
                                response.clear();
                                goto exit_loop;
                            }
                            break;
                        case 3: if (response[idx] != 0xFE) {
                                response.clear();
                                goto exit_loop;
                            }
                            break;
                        case 4: if (response[idx] != *selectedId) {
                                response.clear();
                                goto exit_loop;
                            }
                            break;
                        case 5: if (response[idx] != 0x03) {
                                response.clear();
                                goto exit_loop;
                            }
                            break;
                        case 6: if (response[idx] != request_crc[5]) {
                                response.clear();
                                goto exit_loop;
                            }
                            break;
                        case 7: length = response[6];
                            break;
                    }
                    if (count > 7 && length > 0) {
                        valueCnt++;
                        if (valueCnt == length) {
                            if (!isCrcValid(response)) {
                                response.clear();
                            }
                            if (response.size() > length + 7) {
                                response.erase(response.begin() + length + 7, response.end());
                            }
                            goto exit_loop;
                        }
                    } else if (count >= 7 && length < 1) break;
                }
            } else {
                int now = now_ms();
                if (now - last_read_time > interByteTimeout || now - start_time > totalTimeout) {
                    std::cout << "timeout !" << std::endl;
                    break;
                }
            }
        }
    }
exit_loop:

    if (response.size() > 9 && isCrcValid(response)) {
        result_c = response;
    }

    std::vector<uint8_t> result = {};

    if (result_c.size() > 9) {
        std::cout << getCurrentTimeString() << " (RXD) ";
        for (int i = 0; i < result_c.size(); i++) {
            uint8_t byte = result_c[i];
            std::cout << "0x" << std::hex << std::uppercase << std::setw(2)
                    << std::setfill('0') << static_cast<int>(byte);
            if (i < result_c.size() - 1) std::cout << " ";
        }
        std::cout << std::endl;

        if (result_c.size() == 10) {
            result.push_back(result_c[7]);
        } else {
            result.assign(result_c.begin() + 7, result_c.end() - 2);
        }
    }

    return result;
}

std::vector<uint8_t> Controller::requestWithoutResponse(std::vector<uint8_t> request, std::string description) {
    if (*connector->baudrate < 0) {
        // std::cerr << "Cannot request without baudrate value" << std::endl;
        return {};
    }
    if (connector->selectedPort->length() < 1) {
        // std::cerr << "Cannot request without port" << std::endl;
        return {};
    }

    std::vector<uint8_t> request_crc = request;

    if (request.size() > 3 && request[0] != 0xFF) {
        request_crc = makeRequest(request);
    }

    if (connector->fd == nullptr || *connector->fd < 0 || get_port_path() != *connector->selectedPort) {
        if (*connector->fd > 0) {
            close(*connector->fd);
        }

        connector->connectPort(*connector->selectedPort, *connector->baudrate);
    }
    if (*connector->fd < 0) {
        // std::cerr << "Failed to open port" << std::endl;
        return {};
    }
    write(*connector->fd, request_crc.data(), request_crc.size());
    return {};
}

/*
 * Description: CRC 값을 포함한 모든 요청 데이터 제어기에 전송, 데이터를 전송하는 요청만 사용하세요.
 * Parameter: requests - CRC값 포함된 요청 N개
 */
void Controller::requestAll(std::vector<uint8_t> requests) {
    if (*connector->baudrate < 0) {
        std::cerr << "Cannot request without baudrate value" << std::endl;
        return;
    }
    if (connector->selectedPort->length() < 1) {
        std::cerr << "Cannot request without port" << std::endl;
        return;
    }
#if defined(_WIN32)
    if (connector->selectedPort->rfind("COM", 0) != 0) {
        std::cerr << "Invalid port" << std::endl;
        return;
    }
    if (connector->hComm == nullptr) {
        connector->connectPort(*connector->selectedPort, *connector->baudrate);
    }
    if (connector->hComm == INVALID_HANDLE_VALUE) {
        std::cerr << "Cannot connect to port" << std::endl;
        return;
    }
#elif defined(__linux__)
    if (connector->fd == nullptr || *connector->fd < 0 || get_port_path() != *connector->selectedPort) {
        if (*connector->fd > 0) {
            close(*connector->fd);
        }

        connector->connectPort(*connector->selectedPort, *connector->baudrate);
    }
    if (*connector->fd < 0) {
        std::cerr << "Failed to open port" << std::endl;
        return;
    }
#endif

#if defined(_WIN32)
    WriteFile(connector->hComm, requests.data(), requests.size(), nullptr, nullptr);
#elif defined(__linux__)
    write(*connector->fd, requests.data(), requests.size());
#endif

    std::cout << getCurrentTimeString() << " (TXD) ";
    for (int i = 0; i < requests.size(); i++) {
        uint8_t byte = requests[i];
        std::cout << "0x" << std::hex << std::uppercase << std::setw(2)
                << std::setfill('0') << static_cast<int>(byte);
        if (i < requests.size() - 1) std::cout << " ";
    }
}

std::vector<uint8_t> Controller::makeRequest(std::vector<uint8_t> request) {
    std::vector<uint8_t> request_prefix = {0xFF, 0xFF, 0xFE, *selectedId};
    request.insert(request.begin(), request_prefix.begin(), request_prefix.end());
    const size_t len = request.size();
    uint16_t crc = computeCrc16(request.data(), len);

    // CRC 값 분리 (리틀엔디언)
    uint8_t crcLow = crc & 0x00FF;
    uint8_t crcHigh = (crc >> 8) & 0x00FF;

    // 결과 복사 + CRC 추가
    request.push_back(crcLow);
    request.push_back(crcHigh);

    return request;
}

/*
 * Description: CRC16-MODBUS 값을 데이터 끝에 붙여주는 함수
 * Parameter: data - CRC가 추가되어 있지 않은 데이터
 * Return value: data 끝에 CRC-L, CRC-H 붙여진 바이트 벡터
 */
std::vector<uint8_t> Controller::addCRC16Modbus(std::vector<uint8_t> &data) {
    const size_t len = data.size();
    uint16_t crc = computeCrc16(data.data(), len);

    // CRC 값 분리 (리틀엔디언)
    uint8_t crcLow = crc & 0x00FF;
    uint8_t crcHigh = (crc >> 8) & 0x00FF;

    // 결과 복사 + CRC 추가
    data.push_back(crcLow);
    data.push_back(crcHigh);

    return data;
}

uint16_t Controller::computeCrc16(const uint8_t *data_blk, size_t data_blk_size) {
    unsigned short crc16 = 0xFFFF;
    for (size_t cnt = 0; cnt < data_blk_size; cnt++) {
        crc16 = (unsigned short) ((crc16 >> 8) ^ CrcTable[(crc16 ^ data_blk[cnt]) & 0xFF]);
    }
    return crc16;
}

void Controller::setBaudrate(unsigned int value) {
    uint8_t baudrate_1 = (value >> 24) & 0xFF;
    uint8_t baudrate_2 = (value >> 16) & 0xFF;
    uint8_t baudrate_3 = (value >> 8) & 0xFF;
    uint8_t baudrate_4 = value & 0xFF;
    std::vector<uint8_t> request_v = {
        0x02, Address::BAUDRATE, 0x06, baudrate_1, baudrate_2, baudrate_3, baudrate_4
    };

    request(request_v, "Set baudrate " + std::to_string(value));
    *connector->baudrate = value;
}

/*
 * Description: 데이터의 CRC가 유효한지 검사하는 함수
 * Parameter: data - 검사하고자 하는 1바이트 벡터(마지막 2개 요소 각각 CRC-L, CRC-H)
 * Return value: CRC 유효하면 true, CRC 올바르지 않으면 false
 */
bool Controller::isCrcValid(const std::vector<uint8_t> &data) {
    if (data.size() < 3) return false;
    const size_t len = data.size();

    uint16_t receivedCrc = data[len - 2] | (data[len - 1] << 8);
    uint16_t computedCrc = computeCrc16(data.data(), len - 2);

    return receivedCrc == computedCrc;
}

/*
 * Description: 연결한 포트 제어기의 아이디(1 ~ 15 사이) 찾는 함수
 * 한 포트에 한 개의 제어기만 연결되어 있을 경우에만 유효합니다.
 * Return value: 유효한 아이디, 또는 유효한 아이디가 없을 경우 기본값 0x01 리턴
 */
uint8_t Controller::getId() {

    for (int i = 1; i < 16; i++) {
        std::vector<uint8_t> request_raw = {
            0xFF, 0xFF, 0xFE, static_cast<uint8_t>(i), 0x02, Address::ID, 0x03, 0x01
        };

        std::vector<uint8_t> request_v = addCRC16Modbus(request_raw);

        std::vector<uint8_t> response = request(request_v, "Get ID ");
        if (response.size() == 1 && response[0] == i) {
            std::cout << "ID: " << i << std::endl;
            return response[0];
        }
    }
    return 0x01;
}
