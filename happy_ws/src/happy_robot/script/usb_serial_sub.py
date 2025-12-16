#! /usr/bin/env python3

import serial.tools.list_ports
import serial
import rospy
from geometry_msgs.msg import Twist
import os
import time
import Jetson.GPIO as GPIO

class USBSerial :

    def __init__(self):
        # self.port = '/dev/ttyTHS0'
        self.port = '/dev/ttyUSB2'
        # self.baudrate = 115200
        self.baudrate = 1000000
        self.parity = serial.PARITY_NONE
        self.stopbits = serial.STOPBITS_ONE
        self.bytesize = serial.EIGHTBITS
        self.serial = serial.Serial(port=self.port, baudrate=self.baudrate, parity=self.parity, stopbits=self.stopbits, bytesize=self.bytesize, timeout=1)
        self.serial_ports = serial.tools.list_ports.comports()
        self.seq = 0
        self.LED_PIN = 11
        GPIO.setmode(GPIO.BOARD)  # BOARD 번호 체계 사용
        GPIO.setup(self.LED_PIN, GPIO.OUT, initial=GPIO.LOW)
        GPIO.output(self.LED_PIN, GPIO.LOW)
        self.ping = bytes([0xFF, 0xFE, 0x01, 0x02, 0x01, 0x03, 0x01, 0xAD, 0x59])
        self.ping_hex = b'\xff\xff\xfe\x01\x02\x01\x03\x01\xb9\x56'
        self.ping_9_hex = b'\xff\xff\xfe\x09\x02\x01\x03\x01\x58\x97'
        self.motor_1_8_init_hex = [
            b'\xff\xff\xfe\x01\x01\x18\x03\x06\x29\x17',
            b'\xff\xff\xfe\x02\x01\x18\x03\x06\x6d\x17',
            b'\xff\xff\xfe\x03\x01\x18\x03\x06\x50\xd7',
            b'\xff\xff\xfe\x04\x01\x18\x03\x06\xe5\x17',
            b'\xff\xff\xfe\x05\x01\x18\x03\x06\xd8\xd7',
            b'\xff\xff\xfe\x06\x01\x18\x03\x06\x9c\xd7',
            b'\xff\xff\xfe\x07\x01\x18\x03\x06\xa1\x17',
            b'\xff\xff\xfe\x08\x01\x18\x03\x06\xf5\x16',
            b'\xff\xff\xfe\x09\x01\x18\x03\x06\xc8\xd6'
        ]
        self.motor_1_4_vel_hex = [ 
            b'\xff\xff\xfe\x01\x01\x54\x03\x74\x68\xe5',
            b'\xff\xff\xfe\x02\x01\x54\x03\x74\x2c\xe5',
            b'\xff\xff\xfe\x03\x01\x54\x03\x74\x11\x25',
            b'\xff\xff\xfe\x04\x01\x54\x03\x74\xa4\xe5'
        ]
        self.motor_5_9_pos_hex = [
            b'\xff\xff\xfe\x05\x01\x54\x03\xf4\x98\x85',
            b'\xff\xff\xfe\x06\x01\x54\x03\xf4\xdc\x85',
            b'\xff\xff\xfe\x07\x01\x54\x03\xf4\xe1\x45',
            b'\xff\xff\xfe\x08\x01\x54\x03\xf4\xb5\x44',
            b'\xff\xff\xfe\x09\x01\x54\x03\xf4\x88\x84'
        ]
        self.motor_1_4_acc_dcc_1000_hex = b'\xff\xff\xfe\x00\x01\x50\x06\x03\xe8\x03\xe8\x5c\x3a'
        self.motor_1_4_sub_1000 =         b'\xff\xff\xfe\x00\x01\x55\x05\x00\x03\xe8\x83\x67'
        self.motor_1_4_target_100 =       b'\xff\xff\xfe\x00\x01\x58\x05\x00\x00\x64\xaf\xf3'
        self.motor_5_8_acc_dcc_10_hex =   b'\xff\xff\xfe\x00\x01\x50\x06\x00\x0a\x00\x0a\x7c\xf1'
        self.motor_5_8_sub_100 =          b'\xff\xff\xfe\x00\x01\x55\x05\x00\x03\xe8\x83\x67'
        self.motor_1_4_setup = [self.motor_1_4_acc_dcc_1000_hex, self.motor_1_4_sub_1000, self.motor_1_4_target_100]
        self.motor_1_4_brake = b'\xff\xff\xfe\x01\x01\x5d\x03\x00\xb8\xc0\xff\xff\xfe\x02\x01\x5d\x03\x00\xfc\xc0\xff\xff\xfe\x03\x01\x5d\x03\x00\xc1\x00\xff\xff\xfe\x04\x01\x5d\x03\x00\x74\xc0'
        self.motor_1_4_nonBrake = b'\xff\xfe\x01\x01\x5d\x03\x01\x6d\x0f\xff\xfe\x02\x01\x5d\x03\x01\x29\x0f\xff\xfe\x03\x01\x5d\x03\x01\x14\xcf\xff\xfe\x04\x01\x5d\x03\x01\xa1\x0f'
        self.motor_1_4_value_0_hex = b'\xff\xfe\x01\x01\x5b\x03\x00\x4c\xce\xff\xfe\x02\x01\x5b\x03\x00\x08\xce\xff\xfe\x03\x01\x5b\x03\x00\x35\x0e\xff\xfe\x04\x01\x5b\x03\x00\x80\xce'
        self.motor_1_4_forward = b'\xff\xfe\x01\x01\x5b\x03\x01\x8d\x0e\xff\xfe\x02\x01\x5b\x03\x02\x89\x0f\xff\xfe\x03\x01\x5b\x03\x01\xf4\xce\xff\xfe\x04\x01\x5b\x03\x02\x01\x0f'
        self.motor_1_4_backward = b'\xff\xfe\x01\x01\x5b\x03\x02\xcd\x0f\xff\xfe\x02\x01\x5b\x03\x01\xc9\x0e\xff\xfe\x03\x01\x5b\x03\x02\xb4\xcf\xff\xfe\x04\x01\x5b\x03\x01\x41\x0e'


        # self.motor_5_8_acc_dcc_10_hex = b'\xff\xff\xfe\x00\x01\x50\x06\x00\x0a\x00\x0a\x7c\xf1'
        # self.motor_5_8_sub_100 = b'\xff\xff\xfe\x00\x01\x55\x05\x00\x03\xe8\x83\x67'
        self.motor_5_8_setup = [self.motor_5_8_acc_dcc_10_hex, self.motor_5_8_sub_100]
        self.motor_5_8_brake = b'\xff\xff\xfe\x05\x01\x5d\x03\x00\x5d\x0f\xff\xfe\x06\x01\x5d\x03\x00\x19\x0f\xff\xfe\x07\x01\x5d\x03\x00\x24\xcf\xff\xfe\x08\x01\x5d\x03\x00\x70\xce'
        self.motor_5_8_nonBrake = b'\xff\xff\xfe\x05\x01\x5d\x03\x01\x9c\xcf\xff\xfe\x06\x01\x5d\x03\x01\xd8\xcf\xff\xfe\x07\x01\x5d\x03\x01\xe5\x0f\xff\xfe\x08\x01\x5d\x03\x01\xb1\x0e'
        self.motor_5_8_vertical_hex = b'\xff\xff\xfe\x05\x01\x5B\x03\x00\xBD\x0E\xff\xfe\x06\x01\x5B\x03\x00\xF9\x0E\xff\xfe\x07\x01\x5B\x03\x00\xC4\xCE\xff\xfe\x08\x01\x5B\x03\x00\x90\xCF'
        self.motor_5_8_horizontal_hex = b'\xff\xff\xfe\x05\x01\x58\x05\x00\x03\x84\xB4\x8F\xff\xfe\x06\x01\x58\x05\x00\x03\x84\x87\x8F\xff\xfe\x07\x01\x58\x05\x00\x03\x84\x97\x4F\xff\xfe\x08\x01\x58\x05\x00\x03\x84\x68\x4F\xff\xfe\x05\x01\x5B\x03\x01\x7C\xCE\xff\xfe\x06\x01\x5B\x03\x02\x78\xCF\xff\xfe\x07\x01\x5B\x03\x02\x45\x0F\xff\xfe\x08\x01\x5B\x03\x01\x51\x0F'
        self.motor_5_8_rotational_hex = b'\xff\xff\xfe\x05\x01\x58\x05\x00\x02\x35\x3a\x6f\xff\xff\xfe\x06\x01\x58\x05\x00\x02\x35\x09\x6f\xff\xff\xfe\x07\x01\x58\x05\x00\x02\x35\x19\xaf\xff\xff\xfe\x08\x01\x58\x05\x00\x02\x35\xe6\xaf\xff\xff\xfe\x05\x01\x5b\x03\x01\x68\xc1\xff\xff\xfe\x06\x01\x5b\x03\x02\x6c\xc0\xff\xff\xfe\x07\x01\x5b\x03\x02\x51\x00\xff\xff\xfe\x08\x01\x5b\x03\x01\x45\x00'

        self.motor_all_only_0 = b'\xff\xff\xfe\x00\x01\x58\x06\x00\x00\x00\x01\x59\x88'
        self.motor_1_0 = b'\xff\xff\xfe\x01\x01\x58\x06\x00\x00\x00\x01\x98\x44'
        self.motor_1_only_forward = b'\xff\xff\xfe\x01\x01\x5b\x03\x01\x8d\x0e'
        self.motor_1_only_backward = b'\xff\xff\xfe\x01\x01\x5b\x03\x02\xcd\x0f'
        self.motor_1_get_rpm = b'\xff\xff\xfe\x01\x02\x6A\x03\x03\x5d\x44'

    def ping_test(self, data):
        GPIO.output(self.LED_PIN, GPIO.HIGH)
        time.sleep(0.00001)
        self.serial.write(data)
        time.sleep(0.00001)  # 2초 대기
        GPIO.output(self.LED_PIN, GPIO.LOW)
        print(f"Sent: {data}")
        # self.send_horizontal_signal()
        time.sleep(0.5)  # 응답 대기 (필요시 조정)
        response = self.serial.read(self.serial.in_waiting or 1)  # 받은 데이터 읽기
        if response:
            print(f"Received: {response.hex().upper()}")
        else:
            print("No response received.")

    def input_iternal(self):
        rospy.on_shutdown(self.myhook)
        while not rospy.is_shutdown() :
            a = input('input command : ')
            if (a == 'p'):
                print(f"키 눌림: a, ping test")
                self.ping_test(self.ping_hex)
            elif (a in [str(i) for i in range(1, 10)]):
                print(f"키 눌림: a, {a}")
                self.ping_test(self.motor_1_8_init_hex[int(a)-1])
            elif (a == '0'):
                print(f"키 눌림: a, {a}")
                self.init_setup()
            elif(a == 'a'):
                print(f"키 눌림: a, 주행 모터 논브레이크")
                self.ping_test(self.motor_1_4_nonBrake)
            elif (a == 's'):
                print(f"키 눌림: s, 주행 모터 브레이크")
                self.ping_test(self.motor_1_4_brake)
            elif(a == 'f'):
                print(f"키 눌림: a, 조향 모터 논브레이크")
                self.ping_test(self.motor_5_8_nonBrake)
            elif (a == 'g'):
                print(f"키 눌림: s, 조향 모터 브레이크")
                self.ping_test(self.motor_5_8_brake)
            elif (a == 'q'):
                print(f"키 눌림: s, 전진")
                self.ping_test(self.motor_1_4_forward)
            elif (a == 'w'):
                print(f"키 눌림: s, 후진")
                self.ping_test(self.motor_1_4_backward)
            elif (a == 'z'):
                print(f"키 눌림: s, vertical")
                self.ping_test(self.motor_5_8_vertical_hex)
            elif (a == 'x'):
                print(f"키 눌림: s, rotational")
                self.ping_test(self.motor_5_8_rotational_hex)
            elif (a == 'c'):
                print(f"키 눌림: s, horizontal")
                self.ping_test(self.motor_5_8_horizontal_hex)
            elif (a == 'd'):
                print(f"키 눌림: d, 종료")
                os._exit(0)
            elif (a == 'i'):
                print(f"키 눌림: i, rpm 1")
                self.ping_test(self.motor_1_get_rpm)


    def init_setup(self):

        rospy.loginfo("init setup")
        self.print_ports()
        try:
            data = self.ping_hex
            GPIO.output(self.LED_PIN, GPIO.HIGH)
            time.sleep(0.00001)
            self.serial.write(data)
            time.sleep(0.00001)  # 2초 대기
            GPIO.output(self.LED_PIN, GPIO.LOW)
            print(f"Sent: {data}")
            # self.send_horizontal_signal()
            time.sleep(0.5)  # 응답 대기 (필요시 조정)
            response = self.serial.read(self.serial.in_waiting or 1)  # 받은 데이터 읽기
            if response:
                print(f"Received: {response.hex().upper()}")
            else:
                print("No response received.")

            for hex in self.motor_1_4_vel_hex :
                GPIO.output(self.LED_PIN, GPIO.HIGH)
                time.sleep(0.00001)
                self.serial.write(hex)
                time.sleep(0.00001)  # 2초 대기
                GPIO.output(self.LED_PIN, GPIO.LOW)
                time.sleep(0.5)
            rospy.loginfo("complete motor_1_4_vel_hex")

            for hex in self.motor_5_9_pos_hex :
                GPIO.output(self.LED_PIN, GPIO.HIGH)
                time.sleep(0.00001)
                self.serial.write(hex)
                time.sleep(0.00001)  # 2초 대기
                GPIO.output(self.LED_PIN, GPIO.LOW)
                time.sleep(0.5)
            rospy.loginfo("complete motor_5_8_pos_hex")

            for hex in self.motor_1_4_setup :
                GPIO.output(self.LED_PIN, GPIO.HIGH)
                time.sleep(0.00001)
                self.serial.write(hex)
                time.sleep(0.00001)  # 2초 대기
                GPIO.output(self.LED_PIN, GPIO.LOW)
                time.sleep(0.5)
            rospy.loginfo("complete motor_1_4_setup")  

            for hex in self.motor_5_8_setup :
                GPIO.output(self.LED_PIN, GPIO.HIGH)
                time.sleep(0.00001)
                self.serial.write(hex)
                time.sleep(0.00001)  # 2초 대기
                GPIO.output(self.LED_PIN, GPIO.LOW)
                time.sleep(0.5)
            rospy.loginfo("complete motor_5_8_setup") 

            rospy.loginfo("complete init setup !")
        except serial.SerialException as e:
            print(f"Serial error: {e}")
        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            if 'ser' in locals() and self.serial.is_open:
                self.serial.close()
                print("Serial port closed.")


    def get_ports(self):
        return self.serial_ports

    def print_ports(self):
        for ports in self.serial_ports:
            print(ports)

    def myhook(self):
        print ("shutdown time!")
        os._exit(0)

    def processing(self):
        self.init_setup()
        os._exit(0)
        rospy.on_shutdown(self.myhook)
        

if __name__ == "__main__":
    try:
        rospy.init_node("init_usb_serial_sub_node")
        rospy.loginfo("init_usb_serial_sub_node")
        usb_serial = USBSerial()
        usb_serial.input_iternal()
        # usb_serial.processing()

    except Exception as e:
        print("Error: ", e)
