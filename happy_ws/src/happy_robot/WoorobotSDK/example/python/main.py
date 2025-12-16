import woorobot_sdk_python._core as woorobot
import time

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

        # 모터 run
        controller.motor_control.set_acc_time(1000)
        controller.motor_control.set_dec_time(1000)
        controller.motor_control.set_motion_mode(0xF4)
        controller.motor_control.set_sub_target(10000)
        controller.motor_control.set_main_target(3600)
        controller.motor_control.set_motor_direction(0x01)

        time.sleep(1)
        controller.motor_control.set_acc_time(1000)
        controller.motor_control.set_dec_time(1000)
        controller.motor_control.set_motion_mode(0xF4)
        controller.motor_control.set_sub_target(10000)
        controller.motor_control.set_main_target(3600)
        controller.motor_control.set_motor_direction(0x00)

        time.sleep(1)
        controller.motor_control.set_acc_time(1000)
        controller.motor_control.set_dec_time(1000)
        controller.motor_control.set_motion_mode(0xF4)
        controller.motor_control.set_sub_target(10000)
        controller.motor_control.set_main_target(3600)
        controller.motor_control.set_motor_direction(0x02)

        time.sleep(1)
        controller.motor_control.set_acc_time(1000)
        controller.motor_control.set_dec_time(1000)
        controller.motor_control.set_motion_mode(0xF4)
        controller.motor_control.set_sub_target(10000)
        controller.motor_control.set_main_target(3600)
        controller.motor_control.set_motor_direction(0x00)

        time.sleep(1)
        controller.motor_control.set_acc_time(1000)
        controller.motor_control.set_dec_time(1000)
        controller.motor_control.set_motion_mode(0xF4)
        controller.motor_control.set_sub_target(10000)
        controller.motor_control.set_main_target(3600)
        controller.motor_control.set_motor_direction(0x01)

        time.sleep(1)
        controller.motor_control.set_acc_time(1000)
        controller.motor_control.set_dec_time(1000)
        controller.motor_control.set_motion_mode(0xF4)
        controller.motor_control.set_sub_target(10000)
        controller.motor_control.set_main_target(3600)
        controller.motor_control.set_motor_direction(0x00)

        time.sleep(1)
        controller.motor_control.set_acc_time(1000)
        controller.motor_control.set_dec_time(1000)
        controller.motor_control.set_motion_mode(0xF4)
        controller.motor_control.set_sub_target(10000)
        controller.motor_control.set_main_target(3600)
        controller.motor_control.set_motor_direction(0x02)

        time.sleep(1)
        controller.motor_control.set_acc_time(1000)
        controller.motor_control.set_dec_time(1000)
        controller.motor_control.set_motion_mode(0xF4)
        controller.motor_control.set_sub_target(10000)
        controller.motor_control.set_main_target(3600)
        controller.motor_control.set_motor_direction(0x00)

        # 연결 해제
        connector.disconnect()
        print("Disconnected.")
    else:
        print("Failed to connect.")


if __name__ == "__main__":
    main()
