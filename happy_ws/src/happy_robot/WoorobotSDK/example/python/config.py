import woorobot_sdk_python._core as woorobot

def main():
    connector = woorobot.SerialPortConnector()

    # 사용 가능한 시리얼 포트 출력
    ports = connector.get_ports()
    print("Available Ports:", ports)

    if not ports:
        print("No ports found.")
        return

    # 포트에 연결 시도 (포트 이름, baudrate)
    success = connector.connect_port(ports[0], 9600)

    if success:
        print(f"Connected to {ports[0]} at 115200.")

        # ID 1번 컨트롤러 추가
        controller = connector.add_controller(1)
        print("Controller added:", controller)

        # 모터 셋팅
        controller.motor_config.set_motor_type(0x31)
        controller.motor_config.set_motor_encoder_direction(0x01)
        controller.motor_config.set_motor_encoder_cpr(4000)
        controller.motor_config.set_electronic_angle(4)
        controller.motor_config.set_gear_ratio(20.0)

        controller.motor_param.set_resistance_constant(1800)
        controller.motor_param.set_inductance_constant(540)
        controller.motor_param.set_torque_constant(42)
        controller.motor_param.set_moment_of_inertia(24)
        controller.motor_param.set_rated_speed(4000.0)
        controller.motor_param.set_rated_current(200.0)
        controller.motor_param.set_rated_voltage(2.4)

        # 연결 해제
        connector.disconnect()
        print("Disconnected.")
    else:
        print("Failed to connect.")


if __name__ == "__main__":
    main()
