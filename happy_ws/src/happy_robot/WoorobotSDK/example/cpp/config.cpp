#include <SerialPortConnector.h>

int main() {
    SerialPortConnector connector = SerialPortConnector();
    std::cout << "연결 가능한 포트" << std::endl;
    for (const auto& port : connector.allPorts) {
        std::cout << " - " << port << std::endl;
    }
    if (connector.allPorts.empty()) {
        return 0;
    }
    connector.connectPort(connector.allPorts[0], 9600);

    std::cout << "Selected port : " << connector.selectedPort->data() << std::endl;

    Controller controller = connector.addController(1);

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