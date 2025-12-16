#! /usr/bin/env python3

import serial.tools.list_ports
import serial
import rospy
from std_msgs.msg import Bool

class ModeConverter :

    def __init__(self):
        # self.port = '/dev/ttyACM0'
        self.port = '/dev/ttyUSB0'
        self.baudrate = 9600
        self.parity = serial.PARITY_NONE
        self.stopbits = serial.STOPBITS_ONE
        self.bytesize = serial.EIGHTBITS
        self.serial = serial.Serial(port=self.port, baudrate=self.baudrate, parity=self.parity, stopbits=self.stopbits, bytesize=self.bytesize, timeout=1)
        self.serial_ports = serial.tools.list_ports.comports()
        self.seq = 0
        self.current_control_mode = True
        self.control_mode = "/control_mode"
        self.control_mode_sub = rospy.Subscriber(self.control_mode, Bool, self.control_mode_callback)
        self.init_pub = rospy.Publisher("/init", Bool, queue_size=10)
        self.motor_1_4_init_hex = [
            b'\xff\xfe\x01\x01\x18\x03\x06\x3d\x18',
            b'\xff\xfe\x02\x01\x18\x03\x06\x79\x18',
            b'\xff\xfe\x03\x01\x18\x03\x06\x44\xd8',
            b'\xff\xfe\x04\x01\x18\x03\x06\xf1\x18'
        ]
        self.motor_1_4_vel_hex = [ 
            b'\xff\xfe\x01\x01\x54\x03\x31\xbd\x19',
            b'\xff\xfe\x02\x01\x54\x03\x31\xf9\x19',
            b'\xff\xfe\x03\x01\x54\x03\x31\xc4\xd9',
            b'\xff\xfe\x04\x01\x54\x03\x31\x71\x19'
        ]
        self.motor_5_8_init_hex = [
            b'\xff\xfe\x05\x01\x18\x03\x06\xcc\xd8',
            b'\xff\xfe\x06\x01\x18\x03\x06\x88\xd8',
            b'\xff\xfe\x07\x01\x18\x03\x06\xb5\x18',
            b'\xff\xfe\x08\x01\x18\x03\x06\xe1\x19'
        ]
        self.motor_5_8_pos_hex = [
            b'\xff\xfe\x05\x01\x54\x03\x71\x4d\x29',
            b'\xff\xfe\x06\x01\x54\x03\x71\x09\x29',
            b'\xff\xfe\x07\x01\x54\x03\x71\x34\xe9',
            b'\xff\xfe\x08\x01\x54\x03\x71\x60\xe8'
        ]

    def initialize(self):
        rospy.loginfo("Initiate Motors")
        while True:
            self.serial.write(b'ff')
            rospy.loginfo("send")
            rospy.sleep(1)
        self.init_pub.publish(True)
        rospy.spin()

    def control_mode_callback(self, msg):
        rospy.loginfo("Control Mode Callback")
        if (self.current_control_mode == msg.data):
            rospy.logerr("Same Control Mode")
            rospy.signal_shutdown("Same Control Mode")
        self.current_control_mode = msg.data
        rospy.loginfo("Convert Control Mode")
        self.send_mode_data()

    def send_vertical_signal(self):
        # self.serial.write(b'\xff\xfe\x05\x01\x5B\x03\x00\xBD\x0E')
        self.serial.write(b'\xff\xfe\x05\x01\x5B\x03\x00\xBD\x0E\xff\xfe\x06\x01\x5B\x03\x00\xF9\x0E\xff\xfe\x07\x01\x5B\x03\x00\xC4\xCE\xff\xfe\x08\x01\x5B\x03\x00\x90\xCF')

    def send_horizontal_signal(self):
        self.serial.write(b'\xff\xfe\x05\x01\x58\x05\x00\x03\x84\xB4\x8F\xff\xfe\x06\x01\x58\x05\x00\x03\x84\x87\x8F\xff\xfe\x07\x01\x58\x05\x00\x03\x84\x97\x4F\xff\xfe\x08\x01\x58\x05\x00\x03\x84\x68\x4F\xff\xfe\x05\x01\x5B\x03\x01\x7C\xCE\xff\xfe\x06\x01\x5B\x03\x02\x78\xCF\xff\xfe\x07\x01\x5B\x03\x02\x45\x0F\xff\xfe\x08\x01\x5B\x03\x01\x51\x0F')

    def send_mode_data(self):
        if (self.current_control_mode): 
            self.send_vertical_signal()
            rospy.loginfo("Send Vertical Signal : %d", self.seq)
        else : 
            self.send_horizontal_signal()
            rospy.loginfo("Send Horizontal Signal : %d", self.seq)
        self.seq +=1 

    def send_motor_data(self, data):
        self.serial.write(data)

    def print_ports(self):
        for ports in self.serial_ports:
            print(ports)

if __name__ == "__main__":
    try:
        rospy.init_node("init_mode_converter_node")
        rospy.loginfo("init_mode_converter_node")
        modeConverter = ModeConverter()
        modeConverter.print_ports()
        modeConverter.initialize()

    except Exception as e:
        print("Error: ", e)
