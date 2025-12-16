#include "Firmware.h"
#include "../include/Controller.h"
#include "SerialPortConnector.h"

Firmware::Firmware(Controller *controller) : controller(controller) {
}

uint8_t Firmware::hexCharToByte(char c) {
    if ('0' <= c && c <= '9') return c - '0';
    if ('A' <= c && c <= 'F') return c - 'A' + 10;
    if ('a' <= c && c <= 'f') return c - 'a' + 10;
    return 0;
}

uint8_t Firmware::hexPairToByte(char high, char low) {
    return (hexCharToByte(high) << 4) | hexCharToByte(low);
}

void printProgressBar(int current, int total) {
    const int barWidth = 50;
    float progress = (float)current / total;
    int pos = barWidth * progress;

    std::cout << "\r[";

    // 색상 시작 (초록색)
    std::cout << "\033[1;32m";
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    // 색상 종료 (기본색)
    std::cout << "\033[0m";

    std::cout << "] " << std::setw(3) << int(progress * 100.0) << "%";
    std::cout.flush();
}

/*
 * Description: 제어기의 펌웨어를 업데이트 하는 함수입니다. 실패 시 runtime error 를 던질 수 있습니다. try-catch 문으로 감싸는 것을 권장합니다.
 * Parameter: file_path - 펌웨어 파일 경로, 예) "/home/example/STM32G.hex"
 * 파일 경로를 찾지 못할 경우 /tmp 폴더 내부로 파일을 복사 후 시도하세요.
 * 지원되는 확장자 : intel hex(.hex)
 */
void Firmware::firmwareUpdate(const char* file_path) {
    std::cout << "Firmware update started" << std::endl;

    std::ifstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Firmware update failed: fd open failed");
    }

    unsigned int address = 0x08002000;
    std::vector<uint8_t> address_bytes;
    std::vector<uint8_t> binaryData;

    std::string line;
    uint32_t extendedAddress = 0;
    uint32_t lastFullAddress = 0xFFFFFFFF;

    std::cout << "Reading file..." << std::endl;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] != ':') continue;

        uint8_t byteCount = hexPairToByte(line[1], line[2]);
        uint16_t addr = (hexPairToByte(line[3], line[4]) << 8) | hexPairToByte(line[5], line[6]);
        uint8_t recordType = hexPairToByte(line[7], line[8]);

        if (recordType == 0x00) {
            uint32_t fullAddress = extendedAddress + addr;

            // 누락된 주소 구간을 0xFF로 채움
            if (lastFullAddress != 0xFFFFFFFF && fullAddress > lastFullAddress) {
                for (uint32_t pad = lastFullAddress; pad < fullAddress; ++pad) {
                    binaryData.push_back(0xFF);
                }
            }

            // 데이터 레코드
            for (size_t i = 0; i < byteCount; ++i) {
                uint8_t byte = hexPairToByte(line[9 + i * 2], line[9 + i * 2 + 1]);
                binaryData.push_back(byte);
            }

            lastFullAddress = fullAddress + byteCount;
        } else if (recordType == 0x04) {
            // 확장 선형 주소
            extendedAddress = (hexPairToByte(line[9], line[10]) << 8) | hexPairToByte(line[11], line[12]);
            extendedAddress <<= 16;
        } else if (recordType == 0x01) {
            // EOF
            break;
        }
    }
    std::cout << "File read complete" << std::endl;

    // std::vector<uint8_t> initRequest = { 0x01, Address::FIRMWARE_VERSION, 0x04, 0xFF, 0xFF };
    // controller->request(initRequest, "Remove firmware");
    // controller->boardConfig.softwareReset();

    file.close();

    std::cout << "Start send firmware" << std::endl;
    // 16 바이트 단위 전송
    for (size_t offset = 0; offset < binaryData.size(); offset += 128) {
        address_bytes.clear();
        for (int i = 3; i >= 0; --i) {
            uint8_t byte = (address >> (i * 8)) & 0xFF;
            address_bytes.push_back(byte);
        }

        size_t bytesToSend = std::min<size_t>(128, binaryData.size() - offset);

        std::vector<uint8_t> request = {0x01};
        request.insert(request.end(), address_bytes.begin(), address_bytes.end());
        request.push_back(static_cast<uint8_t>(bytesToSend) + 2);

        std::vector<uint8_t> requestData = {};

        for (size_t i = 0; i < bytesToSend / 8; ++i) {
            for (size_t j = 8; j > 0; --j) {
                requestData.push_back(binaryData[offset + i * 8 + j - 1]);
            }
        }
        request.insert(request.end(), requestData.begin(), requestData.end());

        requestBootLoader(request);

        request.clear();
        request = {0x02};
        request.insert(request.end(), address_bytes.begin(), address_bytes.end());
        request.push_back(0x03);
        request.push_back(static_cast<uint8_t>(bytesToSend));

        std::vector<uint8_t> response = requestBootLoader(request);
        // if (response.empty()) {
        //     throw std::runtime_error("Firmware update failed : response is empty");
        // }

        // 데이터 write한 값과 응답값 같은지 확인
        // if (response.size() != requestData.size()) {
        //     throw std::runtime_error("Firmware update failed : size is invalid");
        // }
        // for (size_t i = 0; i < requestData.size(); ++i) {
        //     if (response[i] != requestData[i]) {
        //         throw std::runtime_error("Firmware update failed");
        //     }
        // }

        address += bytesToSend;
        // printProgressBar(static_cast<int>(offset + 128), static_cast<int>(binaryData.size()));
    }

    std::vector<uint8_t> request = {0x01, 0x08, 0x01, 0xFF, 0xC0, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    requestBootLoader(request);

    std::cout << "Firmware update complete" << std::endl;
}

std::vector<uint8_t> Firmware::requestBootLoader(std::vector<uint8_t> request) {
    SerialPortConnector *connector = controller->connector;
    if (*connector->baudrate < 0) {
        std::cerr << "Cannot request without baudrate value" << std::endl;
        return {};
    }
    if (connector->selectedPort->length() < 1) {
        std::cerr << "Cannot request without port" << std::endl;
        return {};
    }

    std::vector<uint8_t> request_prefix = {0xFF, 0xFF, 0xFD, *controller->selectedId};
    request.insert(request.begin(), request_prefix.begin(), request_prefix.end());

#if defined(_WIN32)
    if (connector->selectedPort->rfind("COM", 0) != 0) {
        std::cerr << "Invalid port" << std::endl;
        return {};
    }
    if (connector->hComm == nullptr) {
        connector->connectPort(*connector->selectedPort, *connector->baudrate);
    }
    if (connector->hComm == INVALID_HANDLE_VALUE) {
        std::cerr << "Cannot connect to port" << std::endl;
        return {};
    }
#elif defined(__linux__)
    if (connector->fd == nullptr || *connector->fd < 0 || controller->get_port_path() != *connector->
        selectedPort) {
        if (*connector->fd > 0) {
            close(*connector->fd);
        }

        connector->connectPort(*connector->selectedPort, *connector->baudrate);
    }
    if (*connector->fd < 0) {
        std::cerr << "Failed to open port" << std::endl;
        return {};
    }
#endif

    const auto request_crc = controller->addCRC16Modbus(request);
#if defined(_WIN32)
    WriteFile(connector->hComm, request.data(), request.size(), nullptr, nullptr);
#elif defined(__linux__)
    write(*connector->fd, request_crc.data(), request_crc.size());
#endif

    std::cout  << " (TXD) ";
    for (int i = 0; i < request_crc.size(); i++) {
        uint8_t byte = request_crc[i];
        std::cout << "0x" << std::hex << std::uppercase << std::setw(2)
                << std::setfill('0') << static_cast<int>(byte);
        if (i < request_crc.size() - 1) std::cout << " ";
    }
    std::cout << std::endl;

    std::vector<uint8_t> response;
    char buf[256];
    int totalTimeout = 100;
    int interByteTimeout = 100;

    auto now_ms = [this]() {
        timeval tv;
        controller->getTimevalNow(&tv);
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
#if defined(_WIN32)
            DWORD bytesRead = 0;
            BOOL success = ReadFile(connector->hComm, buf, sizeof(buf), &bytesRead, nullptr);
            if (success) {
                n = bytesRead;
            }
#elif defined(__linux__)
            n = read(*connector->fd, buf, sizeof(buf));
#endif
            if (n > 0) {
                response.insert(response.end(), buf, buf + n);

                last_read_time = now_ms();

                // 응답 길이 추적
                // for (int i = 0; i < n; ++i) {
                //     count++;
                //     size_t idx = response.size() - n + i;
                //
                //     switch (count) {
                //         case 1:
                //         case 2: if (response[idx] != 0xFF) {
                //             response.clear();
                //             goto exit_loop;
                //         }
                //             break;
                //         case 3: if (response[idx] != 0xFD) {
                //             response.clear();
                //             goto exit_loop;
                //         }
                //             break;
                //         case 4: if (response[idx] != request_crc[3]) {
                //             response.clear();
                //             goto exit_loop;
                //         } break;
                //         case 5: if (response[idx] != 0x03) {
                //             response.clear();
                //             goto exit_loop;
                //         }
                //             break;
                //         case 6: if (response[idx] != request_crc[5]) {
                //             response.clear();
                //             goto exit_loop;
                //         }
                //             break;
                //         case 7: if (response[idx] != request_crc[6]) {
                //             response.clear();
                //             goto exit_loop;
                //         }
                //             break;
                //         case 8: if (response[idx] != request_crc[7]) {
                //             response.clear();
                //             goto exit_loop;
                //         }
                //             break;
                //         case 9: if (response[idx] != request_crc[8]) {
                //             response.clear();
                //             goto exit_loop;
                //         }
                //             break;
                //         case 10: length = response[9];
                //             break;
                //     }
                //     if (count > 10 && length > 0) {
                //         valueCnt++;
                //         if (valueCnt == length) {
                //             if (!controller->isCrcValid(response)) {
                //                 response.clear();
                //             }
                //             if (response.size() > length + 10) {
                //                 response.erase(response.begin() + length + 10, response.end());
                //             }
                //             goto exit_loop;
                //         }
                //     } else if (count >= 10 && length < 1) goto exit_loop;
                // }
            } else {
                int now = now_ms();
                if (now - last_read_time > interByteTimeout || now - start_time > totalTimeout) goto exit_loop;
            }
        }
    }
    exit_loop:

    std::vector<uint8_t> result = {};

    if (response.size() > 9) {
        std::cout << " (RXD) ";
        for (int i = 0; i < response.size(); i++) {
            uint8_t byte = response[i];
            std::cout << "0x" << std::hex << std::uppercase << std::setw(2)
                    << std::setfill('0') << static_cast<int>(byte);
            if (i < response.size() - 1) std::cout << " ";
        }
        std::cout << std::endl;
        if (response.size() == 11) {
            result.push_back(response[11]);
        } else {
            result.assign(response.begin() + 10, response.end() - 2);
        }
    }

    return result;
}
