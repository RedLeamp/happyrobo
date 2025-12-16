#include "SerialPortConnector.h"

#ifndef BOTHER
#define BOTHER 0010000
#endif

#ifndef TCGETS2
#define TCGETS2 _IOR('T', 0x2A, struct termios2)
#define TCSETS2 _IOW('T', 0x2B, struct termios2)
#endif

SerialPortConnector::SerialPortConnector() {
#if defined(_WIN32)
    SetConsoleOutputCP(CP_UTF8);
#endif
    fd = new int(-1);
    hComm = nullptr;
    baudrate = new unsigned int(0);
    selectedPort = new std::string();
    getPorts();
}

std::vector<std::string> SerialPortConnector::getPorts() {
    allPorts.clear();
#if defined(_WIN32)
    // Windows: COM1 ~ COM256 확인
    for (int i = 1; i <= 256; ++i) {
        std::string portName = "COM" + std::to_string(i);
        std::string devicePath = "\\\\.\\" + portName;

        HANDLE hComm = CreateFile(devicePath.c_str(), GENERIC_READ | GENERIC_WRITE,
                                   0, nullptr, OPEN_EXISTING, 0, nullptr);
        if (hComm != INVALID_HANDLE_VALUE) {
            allPorts.push_back(portName);
            CloseHandle(hComm);
        }
    }

#elif defined(__ANDROID__)
    // Android (Linux 기반): /dev/ttyUSB*, /dev/ttyACM* 등
    namespace fs = std::filesystem;
    const std::string devPath = "/dev";

    for (const auto& entry : fs::directory_iterator(devPath)) {
        const std::string filename = entry.path().filename().string();
        if (filename.find("ttyUSB") == 0 || filename.find("ttyACM") == 0) {
            allPorts.push_back(entry.path().string());
        }
    }

#elif defined(__linux__)
    struct termios2 {
        tcflag_t c_iflag;       /* input mode flags */
        tcflag_t c_oflag;       /* output mode flags */
        tcflag_t c_cflag;       /* control mode flags */
        tcflag_t c_lflag;       /* local mode flags */
        cc_t c_line;            /* line discipline */
        cc_t c_cc[19];          /* control characters */
        speed_t c_ispeed;       /* input speed */
        speed_t c_ospeed;       /* output speed */
    };

    const std::string devPath = "/dev";
    namespace fs = std::filesystem;
    allPorts = std::vector<std::string>();

    for (const auto& entry : std::filesystem::directory_iterator(devPath)) {
        const std::string filename = entry.path().filename().string();

        if (filename.find("ttyUSB") == 0 || filename.find("ttyACM") == 0) {
            allPorts.push_back(entry.path().string());
        }
    }

#elif defined(__APPLE__)
    // macOS: /dev/cu.* 또는 /dev/tty.* 확인
    const std::string devPath = "/dev";
    DIR* dir = opendir(devPath.c_str());
    if (dir != nullptr) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string name(entry->d_name);
            if (name.rfind("cu.", 0) == 0 || name.rfind("tty.", 0) == 0) {
                allPorts.push_back(devPath + "/" + name);
            }
        }
        closedir(dir);
    }

#endif

    return allPorts;
}

bool SerialPortConnector::connectPort(std::string port, unsigned int speed) {
    getPorts();
    bool isContained = std::find(allPorts.begin(), allPorts.end(), port) != allPorts.end();
    if (!isContained || port.empty() || speed == 0) {
        std::cerr << "Port or speed is invalid" << std::endl;
        return false;
    }

    std::cout << "Connecting..." << std::endl;


    if (fd != nullptr && fcntl(*fd, F_GETFL) != -1) {
        close(*fd);
    }
    if (fd == nullptr) {
        fd = new int;
    }

    *fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (*fd < 0) {
        std::cerr << "Cannot connect to port. Check the authority of port" << std::endl;
        return false;
    }

    struct termios2 {
        tcflag_t c_iflag;       /* input mode flags */
        tcflag_t c_oflag;       /* output mode flags */
        tcflag_t c_cflag;       /* control mode flags */
        tcflag_t c_lflag;       /* local mode flags */
        cc_t c_line;            /* line discipline */
        cc_t c_cc[19];          /* control characters */
        speed_t c_ispeed;       /* input speed */
        speed_t c_ospeed;       /* output speed */
    };

    termios2 tio;
    if (ioctl(*fd, TCGETS2, &tio) != 0) return false;
    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER;
    tio.c_ispeed = speed;
    tio.c_ospeed = speed;
    ioctl(*fd, TCSETS2, &tio);

    termios t;
    tcgetattr(*fd, &t);
    t.c_cflag |= (CLOCAL | CREAD);
    t.c_cflag &= ~PARENB;
    t.c_cflag &= ~CSTOPB;
    t.c_cflag &= ~CSIZE;
    t.c_cflag |= CS8;
    t.c_lflag = 0;
    t.c_oflag = 0;
    t.c_iflag = 0;
    // t.c_cc[VTIME] = 10;  // 1초 대기
    t.c_cc[VTIME] = 1;  // 1초 대기
    t.c_cc[VMIN] = 0;
    tcsetattr(*fd, TCSANOW, &t);

    fcntl(*fd, F_SETFL, 0);

    *selectedPort = port;
    *baudrate = speed;

    std::cout << "Connected\n";
    return true;
}

/*
 * Description: 시리얼 포트의 연결을 해제 합니다. 연결이 필요할 경우 connectPort()를 호출하세요.
 */
void SerialPortConnector::disconnect() {
#if defined(_WIN32)
    if (hComm != nullptr) CloseHandle(hComm);

#elif defined(__linux__)
    if (fd != nullptr && fcntl(*fd, F_GETFD) != -1) close(*fd);
    *fd = -1;

#endif
    hComm = nullptr;
    *fd = -1;
    *selectedPort = "";
    *baudrate = 0;
    controllers.clear();
}

// Controller& SerialPortConnector::addController(uint8_t id) {
//     return controllers.emplace_back(id, this);
// }

Controller& SerialPortConnector::addController(uint8_t id) {
    controllers.push_back(std::make_unique<Controller>(id, this));
    return *controllers.back();
}

Controller& SerialPortConnector::getController(uint8_t id) {
    // id는 addController 할 때 등록한 index와 동일해야 함
    // 예: 0 = global, 1..motor_count = 각 모터
    if (id >= controllers.size()) {
        std::cerr << "Invalid controller id :" << id << std::endl;
        throw std::out_of_range("Invalid controller id from getController");
    }
    return *controllers[id];
}
