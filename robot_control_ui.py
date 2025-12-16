#!/usr/bin/env python3
"""
Happy Robot Control UI
웹 기반 로봇 제어 인터페이스
- 실시간 로봇 제어 (전후좌우, 회전)
- 좌우 반복 동작
- 모터 상태 모니터링
- 브레이크 제어
- 모터 초기화
"""

import sys
import os
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QSlider, QGroupBox, QGridLayout, QLineEdit,
    QSpinBox, QDoubleSpinBox, QTextEdit, QTabWidget, QCheckBox,
    QComboBox, QProgressBar
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
from PyQt5.QtGui import QFont, QPalette, QColor
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger, TriggerRequest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

try:
    from happy_robo.srv import TriggerWithCommand, TriggerWithCommandRequest
    CUSTOM_SRV_AVAILABLE = True
except ImportError:
    CUSTOM_SRV_AVAILABLE = False
    print("⚠️  Warning: happy_robo custom services not available")


class ROSNode(QThread):
    """ROS 노드를 별도 스레드에서 실행"""
    odom_signal = pyqtSignal(float, float, float)  # x, y, theta
    laser_signal = pyqtSignal(float)  # min_distance
    
    def __init__(self):
        super().__init__()
        self.running = True
        
    def run(self):
        """ROS 노드 초기화 및 실행"""
        try:
            rospy.init_node('robot_control_ui', anonymous=True)
            
            # Odometry 구독
            rospy.Subscriber('/odom/raw', Odometry, self.odom_callback)
            
            # LaserScan 구독
            rospy.Subscriber('/merged/scan', LaserScan, self.laser_callback)
            
            rate = rospy.Rate(10)  # 10Hz
            while self.running and not rospy.is_shutdown():
                rate.sleep()
                
        except Exception as e:
            print(f"ROS Node Error: {e}")
    
    def odom_callback(self, msg):
        """Odometry 콜백"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Quaternion to Euler
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        
        self.odom_signal.emit(x, y, theta)
    
    def laser_callback(self, msg):
        """LaserScan 콜백"""
        ranges = [r for r in msg.ranges if not math.isnan(r) and not math.isinf(r)]
        if ranges:
            min_dist = min(ranges)
            self.laser_signal.emit(min_dist)
    
    def stop(self):
        """스레드 종료"""
        self.running = False


class RobotControlUI(QMainWindow):
    """로봇 제어 UI 메인 윈도우"""
    
    def __init__(self):
        super().__init__()
        
        # ROS 초기화
        self.init_ros()
        
        # UI 초기화
        self.init_ui()
        
        # 상태 변수
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.auto_repeat_running = False
        self.emergency_stop = False
        
        # 타이머 설정
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)  # 100ms
        
        # 자동 반복 타이머
        self.repeat_timer = QTimer()
        self.repeat_timer.timeout.connect(self.execute_repeat_motion)
        
    def init_ros(self):
        """ROS 퍼블리셔 및 서비스 초기화"""
        # ROS 스레드 시작
        self.ros_thread = ROSNode()
        self.ros_thread.odom_signal.connect(self.update_odom_display)
        self.ros_thread.laser_signal.connect(self.update_laser_display)
        self.ros_thread.start()
        
        # 퍼블리셔
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.stop_mode_pub = rospy.Publisher('/stop_mode', Bool, queue_size=1)
        self.control_mode_pub = rospy.Publisher('/control_mode', Bool, queue_size=1)
        
        # 서비스 프록시
        try:
            rospy.wait_for_service('ping_service', timeout=2.0)
            self.ping_service = rospy.ServiceProxy('ping_service', Trigger)
            
            if CUSTOM_SRV_AVAILABLE:
                self.brake_service = rospy.ServiceProxy('/brake_service', TriggerWithCommand)
                self.motor_init_service = rospy.ServiceProxy('/motor_init_service', TriggerWithCommand)
            
            self.log_message("✅ ROS 서비스 연결 성공")
        except rospy.ROSException as e:
            self.log_message(f"⚠️  ROS 서비스 연결 실패: {e}")
    
    def init_ui(self):
        """UI 초기화"""
        self.setWindowTitle("Happy Robot Control UI")
        self.setGeometry(100, 100, 1200, 800)
        
        # 메인 위젯
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # 메인 레이아웃
        main_layout = QHBoxLayout()
        main_widget.setLayout(main_layout)
        
        # 왼쪽: 제어 패널
        left_panel = self.create_control_panel()
        main_layout.addWidget(left_panel, stretch=2)
        
        # 오른쪽: 모니터링 패널
        right_panel = self.create_monitoring_panel()
        main_layout.addWidget(right_panel, stretch=1)
        
        # 스타일 설정
        self.set_style()
    
    def create_control_panel(self):
        """제어 패널 생성"""
        panel = QWidget()
        layout = QVBoxLayout()
        panel.setLayout(layout)
        
        # 탭 위젯
        tabs = QTabWidget()
        layout.addWidget(tabs)
        
        # 탭 1: 기본 제어
        tab1 = self.create_basic_control_tab()
        tabs.addTab(tab1, "기본 제어")
        
        # 탭 2: 자동 반복
        tab2 = self.create_auto_repeat_tab()
        tabs.addTab(tab2, "자동 반복")
        
        # 탭 3: 고급 설정
        tab3 = self.create_advanced_tab()
        tabs.addTab(tab3, "고급 설정")
        
        # 긴급 정지 버튼
        emergency_btn = QPushButton("🚨 긴급 정지 (Emergency Stop)")
        emergency_btn.setStyleSheet("background-color: red; color: white; font-size: 18px; font-weight: bold; padding: 20px;")
        emergency_btn.clicked.connect(self.emergency_stop_action)
        layout.addWidget(emergency_btn)
        
        return panel
    
    def create_basic_control_tab(self):
        """기본 제어 탭"""
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)
        
        # 속도 설정
        speed_group = QGroupBox("속도 설정")
        speed_layout = QGridLayout()
        speed_group.setLayout(speed_layout)
        
        # 선속도
        speed_layout.addWidget(QLabel("선속도 (m/s):"), 0, 0)
        self.linear_speed_slider = QSlider(Qt.Horizontal)
        self.linear_speed_slider.setMinimum(0)
        self.linear_speed_slider.setMaximum(50)  # 0.5 m/s
        self.linear_speed_slider.setValue(20)
        self.linear_speed_slider.valueChanged.connect(self.update_speed_labels)
        speed_layout.addWidget(self.linear_speed_slider, 0, 1)
        self.linear_speed_label = QLabel("0.20")
        speed_layout.addWidget(self.linear_speed_label, 0, 2)
        
        # 각속도
        speed_layout.addWidget(QLabel("각속도 (rad/s):"), 1, 0)
        self.angular_speed_slider = QSlider(Qt.Horizontal)
        self.angular_speed_slider.setMinimum(0)
        self.angular_speed_slider.setMaximum(50)  # 0.5 rad/s
        self.angular_speed_slider.setValue(20)
        self.angular_speed_slider.valueChanged.connect(self.update_speed_labels)
        speed_layout.addWidget(self.angular_speed_slider, 1, 1)
        self.angular_speed_label = QLabel("0.20")
        speed_layout.addWidget(self.angular_speed_label, 1, 2)
        
        layout.addWidget(speed_group)
        
        # 방향 제어 버튼
        direction_group = QGroupBox("방향 제어")
        direction_layout = QGridLayout()
        direction_group.setLayout(direction_layout)
        
        # 버튼 정의
        buttons = [
            ("↖", 0, 0, "forward_left"),
            ("↑\n전진", 0, 1, "forward"),
            ("↗", 0, 2, "forward_right"),
            ("←\n좌", 1, 0, "left"),
            ("■\n정지", 1, 1, "stop"),
            ("→\n우", 1, 2, "right"),
            ("↙", 2, 0, "backward_left"),
            ("↓\n후진", 2, 1, "backward"),
            ("↘", 2, 2, "backward_right"),
            ("↶\n좌회전", 3, 0, "rotate_left"),
            ("", 3, 1, None),
            ("↷\n우회전", 3, 2, "rotate_right"),
        ]
        
        for label, row, col, direction in buttons:
            if direction:
                btn = QPushButton(label)
                btn.setMinimumSize(100, 80)
                btn.setFont(QFont("Arial", 16, QFont.Bold))
                if direction == "stop":
                    btn.setStyleSheet("background-color: #ff4444; color: white;")
                btn.pressed.connect(lambda d=direction: self.send_direction_command(d, True))
                btn.released.connect(lambda: self.send_direction_command("stop", False))
                direction_layout.addWidget(btn, row, col)
        
        layout.addWidget(direction_group)
        
        return tab
    
    def create_auto_repeat_tab(self):
        """자동 반복 탭"""
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)
        
        # 반복 동작 설정
        repeat_group = QGroupBox("좌우 반복 동작 설정")
        repeat_layout = QGridLayout()
        repeat_group.setLayout(repeat_layout)
        
        # 동작 방향
        repeat_layout.addWidget(QLabel("동작 방향:"), 0, 0)
        self.repeat_direction = QComboBox()
        self.repeat_direction.addItems(["좌우", "전후", "회전"])
        repeat_layout.addWidget(self.repeat_direction, 0, 1)
        
        # 이동 거리/각도
        repeat_layout.addWidget(QLabel("이동 거리 (m):"), 1, 0)
        self.repeat_distance = QDoubleSpinBox()
        self.repeat_distance.setRange(0.1, 10.0)
        self.repeat_distance.setValue(1.0)
        self.repeat_distance.setSingleStep(0.1)
        repeat_layout.addWidget(self.repeat_distance, 1, 1)
        
        # 반복 횟수
        repeat_layout.addWidget(QLabel("반복 횟수:"), 2, 0)
        self.repeat_count = QSpinBox()
        self.repeat_count.setRange(1, 100)
        self.repeat_count.setValue(5)
        repeat_layout.addWidget(self.repeat_count, 2, 1)
        
        # 대기 시간
        repeat_layout.addWidget(QLabel("방향 전환 대기 (초):"), 3, 0)
        self.repeat_delay = QDoubleSpinBox()
        self.repeat_delay.setRange(0.5, 10.0)
        self.repeat_delay.setValue(1.0)
        self.repeat_delay.setSingleStep(0.5)
        repeat_layout.addWidget(self.repeat_delay, 3, 1)
        
        # 속도
        repeat_layout.addWidget(QLabel("속도 (m/s):"), 4, 0)
        self.repeat_speed = QDoubleSpinBox()
        self.repeat_speed.setRange(0.1, 0.5)
        self.repeat_speed.setValue(0.2)
        self.repeat_speed.setSingleStep(0.05)
        repeat_layout.addWidget(self.repeat_speed, 4, 1)
        
        layout.addWidget(repeat_group)
        
        # 진행 상태
        progress_group = QGroupBox("진행 상태")
        progress_layout = QVBoxLayout()
        progress_group.setLayout(progress_layout)
        
        self.repeat_progress = QProgressBar()
        self.repeat_progress.setMaximum(100)
        progress_layout.addWidget(self.repeat_progress)
        
        self.repeat_status_label = QLabel("대기 중...")
        progress_layout.addWidget(self.repeat_status_label)
        
        layout.addWidget(progress_group)
        
        # 제어 버튼
        btn_layout = QHBoxLayout()
        
        self.start_repeat_btn = QPushButton("🚀 반복 시작")
        self.start_repeat_btn.setFont(QFont("Arial", 14, QFont.Bold))
        self.start_repeat_btn.setStyleSheet("background-color: #4CAF50; color: white; padding: 15px;")
        self.start_repeat_btn.clicked.connect(self.start_auto_repeat)
        btn_layout.addWidget(self.start_repeat_btn)
        
        self.stop_repeat_btn = QPushButton("⏹ 반복 중지")
        self.stop_repeat_btn.setFont(QFont("Arial", 14, QFont.Bold))
        self.stop_repeat_btn.setStyleSheet("background-color: #f44336; color: white; padding: 15px;")
        self.stop_repeat_btn.setEnabled(False)
        self.stop_repeat_btn.clicked.connect(self.stop_auto_repeat)
        btn_layout.addWidget(self.stop_repeat_btn)
        
        layout.addLayout(btn_layout)
        layout.addStretch()
        
        return tab
    
    def create_advanced_tab(self):
        """고급 설정 탭"""
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)
        
        # 모터 상태
        motor_group = QGroupBox("모터 상태 및 제어")
        motor_layout = QVBoxLayout()
        motor_group.setLayout(motor_layout)
        
        # Ping 테스트
        ping_btn = QPushButton("🔍 모터 Ping 테스트")
        ping_btn.clicked.connect(self.ping_motors)
        motor_layout.addWidget(ping_btn)
        
        # 모터 초기화
        init_layout = QHBoxLayout()
        init_layout.addWidget(QLabel("모터 ID:"))
        self.motor_id_spin = QSpinBox()
        self.motor_id_spin.setRange(1, 10)
        self.motor_id_spin.setValue(1)
        init_layout.addWidget(self.motor_id_spin)
        
        init_btn = QPushButton("🔧 모터 초기화")
        init_btn.clicked.connect(self.init_motor)
        init_layout.addWidget(init_btn)
        motor_layout.addLayout(init_layout)
        
        # 브레이크 제어
        brake_layout = QHBoxLayout()
        brake_layout.addWidget(QLabel("브레이크:"))
        self.brake_combo = QComboBox()
        self.brake_combo.addItems([
            "0 - Motor Brake",
            "1 - Motor Non-Brake",
            "2 - Outer Brake",
            "3 - Outer Non-Brake",
            "4 - Reset Pose"
        ])
        brake_layout.addWidget(self.brake_combo)
        
        brake_btn = QPushButton("🛑 브레이크 실행")
        brake_btn.clicked.connect(self.control_brake)
        brake_layout.addWidget(brake_btn)
        motor_layout.addLayout(brake_layout)
        
        layout.addWidget(motor_group)
        
        # 주행 모드
        mode_group = QGroupBox("주행 모드")
        mode_layout = QVBoxLayout()
        mode_group.setLayout(mode_layout)
        
        self.vertical_mode_check = QCheckBox("수직 모드 (Vertical Mode)")
        self.vertical_mode_check.setChecked(True)
        self.vertical_mode_check.stateChanged.connect(self.change_control_mode)
        mode_layout.addWidget(self.vertical_mode_check)
        
        self.stop_mode_check = QCheckBox("정지 모드 (Stop Mode)")
        self.stop_mode_check.stateChanged.connect(self.change_stop_mode)
        mode_layout.addWidget(self.stop_mode_check)
        
        layout.addWidget(mode_group)
        
        layout.addStretch()
        
        return tab
    
    def create_monitoring_panel(self):
        """모니터링 패널 생성"""
        panel = QWidget()
        layout = QVBoxLayout()
        panel.setLayout(layout)
        
        # 로봇 상태
        status_group = QGroupBox("로봇 상태")
        status_layout = QGridLayout()
        status_group.setLayout(status_layout)
        
        status_layout.addWidget(QLabel("위치 X:"), 0, 0)
        self.pos_x_label = QLabel("0.00 m")
        status_layout.addWidget(self.pos_x_label, 0, 1)
        
        status_layout.addWidget(QLabel("위치 Y:"), 1, 0)
        self.pos_y_label = QLabel("0.00 m")
        status_layout.addWidget(self.pos_y_label, 1, 1)
        
        status_layout.addWidget(QLabel("방향:"), 2, 0)
        self.theta_label = QLabel("0.00°")
        status_layout.addWidget(self.theta_label, 2, 1)
        
        status_layout.addWidget(QLabel("최소 거리:"), 3, 0)
        self.min_dist_label = QLabel("- m")
        status_layout.addWidget(self.min_dist_label, 3, 1)
        
        layout.addWidget(status_group)
        
        # 현재 명령
        cmd_group = QGroupBox("현재 명령")
        cmd_layout = QGridLayout()
        cmd_group.setLayout(cmd_layout)
        
        cmd_layout.addWidget(QLabel("선속도:"), 0, 0)
        self.cmd_linear_label = QLabel("0.00 m/s")
        cmd_layout.addWidget(self.cmd_linear_label, 0, 1)
        
        cmd_layout.addWidget(QLabel("각속도:"), 1, 0)
        self.cmd_angular_label = QLabel("0.00 rad/s")
        cmd_layout.addWidget(self.cmd_angular_label, 1, 1)
        
        layout.addWidget(cmd_group)
        
        # 로그
        log_group = QGroupBox("로그")
        log_layout = QVBoxLayout()
        log_group.setLayout(log_layout)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(400)
        log_layout.addWidget(self.log_text)
        
        clear_log_btn = QPushButton("로그 지우기")
        clear_log_btn.clicked.connect(self.log_text.clear)
        log_layout.addWidget(clear_log_btn)
        
        layout.addWidget(log_group)
        
        return panel
    
    def set_style(self):
        """스타일 설정"""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f0f0f0;
            }
            QGroupBox {
                font-size: 14px;
                font-weight: bold;
                border: 2px solid #cccccc;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QPushButton {
                font-size: 12px;
                padding: 8px;
                border-radius: 4px;
                background-color: #2196F3;
                color: white;
            }
            QPushButton:hover {
                background-color: #1976D2;
            }
            QPushButton:pressed {
                background-color: #0D47A1;
            }
            QLabel {
                font-size: 12px;
            }
        """)
    
    # ========== 제어 함수 ==========
    
    def send_direction_command(self, direction, pressed):
        """방향 명령 전송"""
        if self.emergency_stop:
            return
        
        twist = Twist()
        linear = self.linear_speed_slider.value() / 100.0
        angular = self.angular_speed_slider.value() / 100.0
        
        if pressed:
            if direction == "forward":
                twist.linear.x = linear
            elif direction == "backward":
                twist.linear.x = -linear
            elif direction == "left":
                twist.linear.y = linear
            elif direction == "right":
                twist.linear.y = -linear
            elif direction == "forward_left":
                twist.linear.x = linear
                twist.angular.z = angular
            elif direction == "forward_right":
                twist.linear.x = linear
                twist.angular.z = -angular
            elif direction == "backward_left":
                twist.linear.x = -linear
                twist.angular.z = -angular
            elif direction == "backward_right":
                twist.linear.x = -linear
                twist.angular.z = angular
            elif direction == "rotate_left":
                twist.angular.z = angular
            elif direction == "rotate_right":
                twist.angular.z = -angular
        
        self.cmd_vel_pub.publish(twist)
        self.current_linear_speed = twist.linear.x
        self.current_angular_speed = twist.angular.z
        
        if pressed and direction != "stop":
            self.log_message(f"🎮 방향: {direction} (선속도: {twist.linear.x:.2f}, 각속도: {twist.angular.z:.2f})")
    
    def emergency_stop_action(self):
        """긴급 정지"""
        self.emergency_stop = True
        twist = Twist()  # 모든 속도 0
        self.cmd_vel_pub.publish(twist)
        self.stop_auto_repeat()
        self.log_message("🚨 긴급 정지 실행!")
        
        QTimer.singleShot(2000, lambda: setattr(self, 'emergency_stop', False))
    
    def start_auto_repeat(self):
        """자동 반복 시작"""
        self.auto_repeat_running = True
        self.auto_repeat_count = 0
        self.auto_repeat_direction = 1  # 1: 정방향, -1: 역방향
        
        self.start_repeat_btn.setEnabled(False)
        self.stop_repeat_btn.setEnabled(True)
        
        self.log_message("🚀 자동 반복 시작")
        self.execute_repeat_motion()
    
    def stop_auto_repeat(self):
        """자동 반복 중지"""
        self.auto_repeat_running = False
        self.repeat_timer.stop()
        
        # 정지
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        self.start_repeat_btn.setEnabled(True)
        self.stop_repeat_btn.setEnabled(False)
        self.repeat_status_label.setText("중지됨")
        self.repeat_progress.setValue(0)
        
        self.log_message("⏹ 자동 반복 중지")
    
    def execute_repeat_motion(self):
        """반복 동작 실행"""
        if not self.auto_repeat_running:
            return
        
        max_count = self.repeat_count.value()
        
        if self.auto_repeat_count >= max_count:
            self.stop_auto_repeat()
            self.log_message("✅ 자동 반복 완료")
            return
        
        # 진행률 업데이트
        progress = int((self.auto_repeat_count / max_count) * 100)
        self.repeat_progress.setValue(progress)
        self.repeat_status_label.setText(f"반복 {self.auto_repeat_count + 1}/{max_count}")
        
        # 동작 실행
        direction_type = self.repeat_direction.currentText()
        speed = self.repeat_speed.value()
        distance = self.repeat_distance.value()
        
        twist = Twist()
        
        if direction_type == "좌우":
            twist.linear.y = speed * self.auto_repeat_direction
            duration = int((distance / speed) * 1000)  # ms
        elif direction_type == "전후":
            twist.linear.x = speed * self.auto_repeat_direction
            duration = int((distance / speed) * 1000)
        else:  # 회전
            twist.angular.z = speed * self.auto_repeat_direction
            duration = int((distance / speed) * 1000)
        
        self.cmd_vel_pub.publish(twist)
        self.log_message(f"➡️ 반복 {self.auto_repeat_count + 1}: {direction_type} {self.auto_repeat_direction}")
        
        # 이동 후 정지
        QTimer.singleShot(duration, self.stop_motion_and_switch)
    
    def stop_motion_and_switch(self):
        """동작 정지 및 방향 전환"""
        # 정지
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # 방향 전환
        self.auto_repeat_direction *= -1
        
        # 카운트 증가 (왕복 1회 = 2카운트)
        if self.auto_repeat_direction == 1:
            self.auto_repeat_count += 1
        
        # 대기 후 다음 동작
        delay = int(self.repeat_delay.value() * 1000)
        QTimer.singleShot(delay, self.execute_repeat_motion)
    
    def ping_motors(self):
        """모터 Ping 테스트"""
        try:
            req = TriggerRequest()
            response = self.ping_service(req)
            results = response.message.split(",")
            
            result_text = "모터 Ping 결과:\n"
            for i, result in enumerate(results):
                status = "✅" if result.strip() == "True" else "❌"
                result_text += f"  Motor {i+1}: {status}\n"
            
            self.log_message(result_text)
        except Exception as e:
            self.log_message(f"❌ Ping 실패: {e}")
    
    def init_motor(self):
        """모터 초기화"""
        if not CUSTOM_SRV_AVAILABLE:
            self.log_message("❌ 모터 초기화 서비스 사용 불가")
            return
        
        try:
            motor_id = self.motor_id_spin.value()
            req = TriggerWithCommandRequest()
            req.command = motor_id - 1  # 0-based index
            response = self.motor_init_service(req)
            
            if response.success:
                self.log_message(f"✅ 모터 {motor_id} 초기화 성공")
            else:
                self.log_message(f"❌ 모터 {motor_id} 초기화 실패")
        except Exception as e:
            self.log_message(f"❌ 모터 초기화 에러: {e}")
    
    def control_brake(self):
        """브레이크 제어"""
        if not CUSTOM_SRV_AVAILABLE:
            self.log_message("❌ 브레이크 서비스 사용 불가")
            return
        
        try:
            command = self.brake_combo.currentIndex()
            req = TriggerWithCommandRequest()
            req.command = command
            response = self.brake_service(req)
            
            if response.success:
                self.log_message(f"✅ 브레이크 명령 {command} 실행 성공")
            else:
                self.log_message(f"❌ 브레이크 명령 {command} 실행 실패")
        except Exception as e:
            self.log_message(f"❌ 브레이크 에러: {e}")
    
    def change_control_mode(self, state):
        """제어 모드 변경"""
        msg = Bool()
        msg.data = (state == Qt.Checked)
        self.control_mode_pub.publish(msg)
        mode = "수직" if msg.data else "수평"
        self.log_message(f"🔄 제어 모드: {mode}")
    
    def change_stop_mode(self, state):
        """정지 모드 변경"""
        msg = Bool()
        msg.data = (state == Qt.Checked)
        self.stop_mode_pub.publish(msg)
        mode = "정지" if msg.data else "동작"
        self.log_message(f"🔄 정지 모드: {mode}")
    
    # ========== 업데이트 함수 ==========
    
    def update_speed_labels(self):
        """속도 라벨 업데이트"""
        linear = self.linear_speed_slider.value() / 100.0
        angular = self.angular_speed_slider.value() / 100.0
        self.linear_speed_label.setText(f"{linear:.2f}")
        self.angular_speed_label.setText(f"{angular:.2f}")
    
    def update_display(self):
        """디스플레이 업데이트"""
        self.cmd_linear_label.setText(f"{self.current_linear_speed:.2f} m/s")
        self.cmd_angular_label.setText(f"{self.current_angular_speed:.2f} rad/s")
    
    def update_odom_display(self, x, y, theta):
        """Odometry 디스플레이 업데이트"""
        self.pos_x_label.setText(f"{x:.2f} m")
        self.pos_y_label.setText(f"{y:.2f} m")
        self.theta_label.setText(f"{math.degrees(theta):.2f}°")
    
    def update_laser_display(self, min_dist):
        """LaserScan 디스플레이 업데이트"""
        self.min_dist_label.setText(f"{min_dist:.2f} m")
        
        # 경고 표시
        if min_dist < 0.5:
            self.min_dist_label.setStyleSheet("color: red; font-weight: bold;")
        else:
            self.min_dist_label.setStyleSheet("")
    
    def log_message(self, message):
        """로그 메시지 추가"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
        # 자동 스크롤
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )
    
    def closeEvent(self, event):
        """종료 이벤트"""
        self.ros_thread.stop()
        self.ros_thread.wait()
        event.accept()


def main():
    """메인 함수"""
    app = QApplication(sys.argv)
    
    # 다크 모드 (선택사항)
    # app.setStyle("Fusion")
    # palette = QPalette()
    # palette.setColor(QPalette.Window, QColor(53, 53, 53))
    # palette.setColor(QPalette.WindowText, Qt.white)
    # app.setPalette(palette)
    
    window = RobotControlUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
