#include <iostream>
#include <SerialPortConnector.h>
#include <thread>
#include <thread>
#include <chrono>

/*
 * 함수 테스트 코드입니다. 사용 시 참고하세요.
 */
void example() {

    SerialPortConnector connector;

    std::cout << connector.allPorts.size() << std::endl;
    std::cout << "연결 가능한 포트" << std::endl;
    for (const auto& port : connector.allPorts) {
        std::cout << " - " << port << std::endl;
    }
    if (connector.allPorts.empty()) {
        return;
    }

    connector.connectPort(connector.allPorts[1], 9600);

    std::cout << "Selected port : " << connector.selectedPort->data() << std::endl;

    Controller& controller = connector.addController(1);

    // 펌웨어 업데이트 (경로 직접 설정)
    // 권한 문제로 file read 불가 시 /tmp 폴더 내부로 복사 후 재시도
    // try {
    //     controller.firmware.firmwareUpdate("/tmp/STM32G431RBT6.hex");
    // } catch (const std::exception& e) {
    //     std::cerr << "Error : " << e.what() << std::endl;
    // }
    //
    // // 필요한 경우 제어기 값으로 SDK 내부 변수 초기화
    // controller.boardConfig.init();
    // controller.mainCommConfig.init();
    // controller.motorConfig.init();
    // controller.motorParam.init();
    // controller.motorGain.init();
    // controller.motorLimit.init();
    // controller.motorControl.init();
    //
    // std::cout << "Error: " << std::dec << static_cast<int>(*controller.error) << "\n"
    // << "max_voltage: " << *controller.max_voltage << "\n"
    // << "min_voltage: " << *controller.min_voltage << "\n"
    // << "max_temperature: " << *controller.max_temperature << std::endl;
    //
    // // connector.mainCommConfig.setWatchdog(10);
    // std::cout << "통신 모드 : " << std::dec << static_cast<int>(*controller.comm_mode) << "\n"
    // << "통신 속도 : " << *connector.baudrate << "\n"
    // << "통신 와치독 : " << static_cast<int>(*controller.watchdog) << "\n" << std::endl;
    //
    // std::cout << std::dec << "모터 종류 : " << static_cast<int>(*controller.motor_type) << "\n"
    // << "모터 엔코더 방향 : " << static_cast<int>(*controller.motor_encoder_direction) << "\n"
    // << "모터 엔코더 속도 : " << static_cast<int>(*controller.motor_encoder_cpr) << "\n"
    // << "전기각 : " << static_cast<int>(*controller.electronic_angle) << "\n"
    // << "기어비 : " << *controller.gear_ratio << "\n"
    // << "초기각 모드 : " << static_cast<int>(*controller.lead_angle_mode) << "\n"
    // << "강제 정렬 : " << static_cast<int>(*controller.force_angle) << "\n"
    // << "초기각 : " << static_cast<int>(*controller.electrical_zero) << "\n"
    // << "홀센서 설정 : " << static_cast<int>(*controller.hallsensor_set) << "\n" << std::endl;
    //
    // std::cout << std::dec << "D 인덕턴스 상수 : " << static_cast<int>(*controller.d_inductance_constant) << "\n"
    // << "Q 인덕턴스 상수 : " << static_cast<int>(*controller.q_inductance_constant) << "\n"
    // << "모터 쇄교 자속 : " << static_cast<int>(*controller.flux_linkage_constant) << "\n"
    // << "모터 저항 : " << static_cast<int>(*controller.resistance_constant) << "\n"
    // << "모터 인덕턴스 : " << static_cast<int>(*controller.inductance_constant) << '\n'
    // << "모터 토크 상수 : " << static_cast<int>(*controller.torque_constant) << '\n'
    // << "모터 관성 모멘트 : " << static_cast<int>(*controller.moment_of_inertia) << '\n'
    // << "정격 속도 : " << *controller.rated_speed << '\n'
    // << "정격 전류 : " << *controller.rated_current << '\n'
    // << "정격 전압 : " << *controller.rated_voltage << "\n" << std::endl;
    //
    // std::cout << std::dec << "position total gain : " << static_cast<int>(*controller.position_total_gain) << "\n"
    // << "velocity total gain : " << static_cast<int>(*controller.velocity_total_gain) << "\n"
    // << "current total gain : " << static_cast<int>(*controller.current_total_gain) << "\n"
    // << "position p gain : " << static_cast<int>(*controller.position_p_gain) << "\n"
    // << "velocity p gain : " << static_cast<int>(*controller.velocity_p_gain) << "\n"
    // << "velocity i gain : " << static_cast<int>(*controller.velocity_i_gain) << "\n"
    // << "current p gain : " << static_cast<int>(*controller.current_p_gain) << "\n"
    // << "current i gain : " << static_cast<int>(*controller.current_i_gain) << "\n" << std::endl;
    //
    // std::cout << std::dec << "모터 최대 시간 : " << *controller.time_max << "\n"
    // << "모터 CCW 최대 위치 : " << std::fixed << std::setprecision(1) << *controller.position_ccw_max << "\n"
    // << "모터 CW 최대 위치 : " << std::fixed << std::setprecision(1) << *controller.position_cw_max << "\n"
    // << "모터 CCW 최대 속도 : " << *controller.velocity_ccw_max << "\n"
    // << "모터 CW 최대 속도 : " << *controller.velocity_cw_max << "\n"
    // << "모터 CCW 최대 전류 : " << *controller.current_ccw_max << "\n"
    // << "모터 CW 최대 전류 : " << *controller.current_cw_max << "\n"
    // << "모터 전류 제한 : " << *controller.current_limit << "\n" << std::endl;

    // 모터 run
    // controller.motorControl.setAccTime(1000);
    // controller.motorControl.setDecTime(1000);
    //
    // controller.motorControl.setMotionMode(0xF5);
    // controller.motorControl.setSubTarget(10000);
    // controller.motorControl.setMainTarget(3600);
    // controller.motorControl.setMotorDirection(0x01);

    std::vector<unsigned int> motorStatus = controller.motorStatus.getMotorStatus();
    unsigned int time_now = motorStatus[0];
    unsigned int time_left = motorStatus[1];
    unsigned int position_now = motorStatus[2];
    int position_dir = motorStatus[3];
    int velocity_now = motorStatus[4];
    int velocity_dir = motorStatus[5];
    int current_now = motorStatus[6];
    int current_dir = motorStatus[7];
    int encoder_now = motorStatus[8];
    int encoder_dir = motorStatus[9];

    std::cout << std::dec << "time now : " << time_now << '\n'
    << "time left : " << time_left << '\n'
    << "position now : " << position_now << '\n'
    << "position dir : " << position_dir << '\n'
    << "velocity now : " << velocity_now << '\n'
    << "velocity dir : " << velocity_dir << '\n'
    << "current now : " << current_now << '\n'
    << "current dir : " << current_dir << '\n'
    << "encoder now : " << encoder_now << '\n'
    << "encoder dir : " << encoder_dir << std::endl;

    connector.disconnect();
}


void configMotor(Controller& controller) {
    controller.motorConfig.setMotorType(0x31);
    controller.motorConfig.setMotorEncoderDirection(0x01);
    controller.motorConfig.setMotorEncoderCpr(4000);
    controller.motorConfig.setElectronicAngle(4);
    controller.motorConfig.setGearRatio(20.0);

    controller.motorParam.setResistanceConstant(1800);
    controller.motorParam.setInductanceConstant(540);
    controller.motorParam.setTorqueConstant(42);
    controller.motorParam.setMomentOfInertia(24);
    controller.motorParam.setRatedSpeed(4000.0);
    controller.motorParam.setRatedCurrent(200.0);
    controller.motorParam.setRatedVoltage(2.4);

    controller.motorConfig.setHallSensorSet(0x01);
    controller.motorConfig.setLeadAngleMode(0x20);
}

void runMotor(Controller& controller, unsigned int degree, uint8_t direction) {
    // 위치, 속도 절대값으로 계산
    controller.motorControl.setAccTime(1000);
    controller.motorControl.setDecTime(1000);

    controller.motorControl.setMotionMode(0xF4);
    controller.motorControl.setSubTarget(10000);
    controller.motorControl.setMainTarget(degree);

    controller.motorControl.setMotorDirection(direction);
}

int main() {
    example();
    // SerialPortConnector connector = SerialPortConnector();
    // std::cout << "연결 가능한 포트" << std::endl;
    // for (const auto& port : connector.allPorts) {
    //     std::cout << " - " << port << std::endl;
    // }
    // if (connector.allPorts.empty()) {
    //     return 0;
    // }
    //
    // connector.connectPort(connector.allPorts[0], 9600);
    // Controller controller = connector.addController(1);
    //
    // runMotor(controller, 3600, 0x01);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // runMotor(controller, 3600, 0x00);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // runMotor(controller, 3600, 0x02);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // runMotor(controller, 3600, 0x00);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // runMotor(controller, 3600, 0x01);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // runMotor(controller, 3600, 0x00);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // runMotor(controller, 3600, 0x02);
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // runMotor(controller, 3600, 0x00);
    // return 0;
}