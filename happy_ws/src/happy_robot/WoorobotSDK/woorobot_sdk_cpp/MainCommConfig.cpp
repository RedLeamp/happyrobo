#include "MainCommConfig.h"
#include "SerialPortConnector.h"


MainCommConfig::MainCommConfig(Controller* controller) : controller(controller) {
}

void MainCommConfig::init() {
    getCommMode();
    getWatchdog();
}

uint8_t MainCommConfig::getCommMode() {
    std::vector<uint8_t> request = makeGetCommModeCommand();
    std::vector<uint8_t> response = controller->request(request, "Get communication mode");
    if (response.size() > 0) {
        *controller->comm_mode = response[0];
    }
    return *controller->comm_mode;
}

void MainCommConfig::setCommMode(uint8_t value) {
    std::vector<uint8_t> request = makeSetCommModeCommand(value);
    controller->request(request, "Set communication mode " + std::to_string(value));
    *controller->comm_mode = value;
}

uint8_t MainCommConfig::getWatchdog() {
    std::vector<uint8_t> request = makeGetWatchdogCommand();
    std::vector<uint8_t> response = controller->request(request, "Get watchdog");
    if (response.size() > 0) {
        *controller->watchdog = response[0];
    }
    return *controller->watchdog;
}

void MainCommConfig::setWatchdog(uint8_t value) {
    std::vector<uint8_t> request = makeSetWatchdogCommand(value);
    controller->request(request, "Set watchdog " + std::to_string(value));
    *controller->watchdog = value;
}

std::vector<uint8_t> MainCommConfig::makeGetCommModeCommand() {
    std::vector<uint8_t> request = {
        0x02, Address::COMMUNICATION_MODE, 0x03, 0x01
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MainCommConfig::makeSetCommModeCommand(uint8_t value) {
    std::vector<uint8_t> request = {
        0x01, Address::COMMUNICATION_MODE, 0x03, value
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MainCommConfig::makeGetWatchdogCommand() {
    std::vector<uint8_t> request = {
        0x02, Address::WATCHDOG, 0x03, 0x01
    };
    return controller->makeRequest(request);
}

std::vector<uint8_t> MainCommConfig::makeSetWatchdogCommand(uint8_t value) {
    std::vector<uint8_t> request = {
        0x01, Address::WATCHDOG, 0x03, value
    };
    return controller->makeRequest(request);
}

void MainCommConfig::setMainCommConfig(uint8_t commMode, uint8_t watchdog) {
    setCommMode(commMode);
    setWatchdog(watchdog);
}
