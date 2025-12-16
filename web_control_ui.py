#!/usr/bin/env python3
"""
Happy Robot Web Control UI
웹 브라우저 기반 로봇 제어 인터페이스 (PyQt5 불필요)
- 실시간 로봇 제어
- 좌우 반복 동작
- 모터 상태 모니터링
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, TriggerRequest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from flask import Flask, render_template_string, jsonify, request
from threading import Thread
import math
import time

try:
    from happy_robo.srv import TriggerWithCommand, TriggerWithCommandRequest
    CUSTOM_SRV_AVAILABLE = True
except ImportError:
    CUSTOM_SRV_AVAILABLE = False
    print("⚠️  Warning: happy_robo custom services not available")

# Flask 앱
app = Flask(__name__)

# 전역 상태 변수
robot_state = {
    'pos_x': 0.0,
    'pos_y': 0.0,
    'theta': 0.0,
    'min_dist': 0.0,
    'current_linear': 0.0,
    'current_angular': 0.0,
    'auto_repeat_running': False,
    'repeat_progress': 0,
    'logs': []
}

# ROS 퍼블리셔
cmd_vel_pub = None
stop_mode_pub = None
control_mode_pub = None

# ROS 서비스
ping_service = None
brake_service = None
motor_init_service = None

def log_message(message):
    """로그 메시지 추가"""
    from datetime import datetime
    timestamp = datetime.now().strftime("%H:%M:%S")
    robot_state['logs'].append(f"[{timestamp}] {message}")
    if len(robot_state['logs']) > 100:
        robot_state['logs'].pop(0)
    print(f"[{timestamp}] {message}")

def odom_callback(msg):
    """Odometry 콜백"""
    robot_state['pos_x'] = msg.pose.pose.position.x
    robot_state['pos_y'] = msg.pose.pose.position.y
    
    # Quaternion to Euler
    quat = msg.pose.pose.orientation
    siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
    robot_state['theta'] = math.atan2(siny_cosp, cosy_cosp)

def laser_callback(msg):
    """LaserScan 콜백"""
    ranges = [r for r in msg.ranges if not math.isnan(r) and not math.isinf(r)]
    if ranges:
        robot_state['min_dist'] = min(ranges)

def ros_thread_func():
    """ROS 스레드"""
    rospy.init_node('web_control_ui', anonymous=True)
    
    # 구독
    rospy.Subscriber('/odom/raw', Odometry, odom_callback)
    rospy.Subscriber('/merged/scan', LaserScan, laser_callback)
    
    rospy.spin()

# HTML 템플릿
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html>
<head>
    <title>Happy Robot Control</title>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: Arial, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            padding: 20px;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            border-radius: 15px;
            padding: 20px;
            box-shadow: 0 10px 40px rgba(0,0,0,0.3);
        }
        h1 {
            text-align: center;
            color: #667eea;
            margin-bottom: 20px;
        }
        .panel {
            display: grid;
            grid-template-columns: 2fr 1fr;
            gap: 20px;
        }
        .control-section, .monitor-section {
            border: 2px solid #e0e0e0;
            border-radius: 10px;
            padding: 15px;
        }
        h2 {
            color: #667eea;
            margin-bottom: 15px;
            border-bottom: 2px solid #667eea;
            padding-bottom: 5px;
        }
        .speed-control {
            margin-bottom: 20px;
        }
        .speed-control label {
            display: block;
            margin-bottom: 5px;
            font-weight: bold;
        }
        .speed-control input[type="range"] {
            width: 100%;
            height: 8px;
        }
        .direction-grid {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            margin-bottom: 20px;
        }
        .btn {
            padding: 20px;
            font-size: 16px;
            font-weight: bold;
            border: none;
            border-radius: 8px;
            cursor: pointer;
            transition: all 0.3s;
            background: #667eea;
            color: white;
        }
        .btn:hover {
            transform: scale(1.05);
            box-shadow: 0 5px 15px rgba(0,0,0,0.3);
        }
        .btn:active {
            transform: scale(0.95);
        }
        .btn-stop {
            background: #ff4444;
        }
        .btn-emergency {
            background: #cc0000;
            grid-column: 1 / -1;
            padding: 30px;
            font-size: 20px;
        }
        .tabs {
            display: flex;
            gap: 10px;
            margin-bottom: 15px;
        }
        .tab {
            flex: 1;
            padding: 10px;
            background: #e0e0e0;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            font-weight: bold;
        }
        .tab.active {
            background: #667eea;
            color: white;
        }
        .tab-content {
            display: none;
        }
        .tab-content.active {
            display: block;
        }
        .auto-repeat-control {
            display: grid;
            gap: 10px;
        }
        .auto-repeat-control label {
            font-weight: bold;
        }
        .auto-repeat-control input, .auto-repeat-control select {
            padding: 8px;
            border: 1px solid #ccc;
            border-radius: 5px;
        }
        .progress-bar {
            width: 100%;
            height: 30px;
            background: #e0e0e0;
            border-radius: 15px;
            overflow: hidden;
            margin: 10px 0;
        }
        .progress-fill {
            height: 100%;
            background: linear-gradient(90deg, #667eea, #764ba2);
            transition: width 0.3s;
            text-align: center;
            line-height: 30px;
            color: white;
            font-weight: bold;
        }
        .status-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
            margin-bottom: 15px;
        }
        .status-item {
            background: #f5f5f5;
            padding: 10px;
            border-radius: 5px;
        }
        .status-label {
            font-size: 12px;
            color: #666;
        }
        .status-value {
            font-size: 18px;
            font-weight: bold;
            color: #333;
        }
        .log-box {
            background: #1e1e1e;
            color: #00ff00;
            padding: 10px;
            border-radius: 5px;
            height: 200px;
            overflow-y: auto;
            font-family: monospace;
            font-size: 12px;
        }
        @media (max-width: 768px) {
            .panel {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Happy Robot Control</h1>
        
        <div class="panel">
            <!-- 제어 패널 -->
            <div class="control-section">
                <div class="tabs">
                    <button class="tab active" onclick="switchTab(0)">기본 제어</button>
                    <button class="tab" onclick="switchTab(1)">자동 반복</button>
                    <button class="tab" onclick="switchTab(2)">고급 설정</button>
                </div>
                
                <!-- 탭 1: 기본 제어 -->
                <div class="tab-content active" id="tab0">
                    <h2>속도 설정</h2>
                    <div class="speed-control">
                        <label>선속도: <span id="linear-speed-val">0.20</span> m/s</label>
                        <input type="range" id="linear-speed" min="0" max="50" value="20" 
                               oninput="updateSpeed()">
                    </div>
                    <div class="speed-control">
                        <label>각속도: <span id="angular-speed-val">0.20</span> rad/s</label>
                        <input type="range" id="angular-speed" min="0" max="50" value="20" 
                               oninput="updateSpeed()">
                    </div>
                    
                    <h2>방향 제어</h2>
                    <div class="direction-grid">
                        <button class="btn" onmousedown="sendCmd('forward_left')" onmouseup="sendCmd('stop')">↖</button>
                        <button class="btn" onmousedown="sendCmd('forward')" onmouseup="sendCmd('stop')">↑<br>전진</button>
                        <button class="btn" onmousedown="sendCmd('forward_right')" onmouseup="sendCmd('stop')">↗</button>
                        
                        <button class="btn" onmousedown="sendCmd('left')" onmouseup="sendCmd('stop')">←<br>좌</button>
                        <button class="btn btn-stop" onclick="sendCmd('stop')">■<br>정지</button>
                        <button class="btn" onmousedown="sendCmd('right')" onmouseup="sendCmd('stop')">→<br>우</button>
                        
                        <button class="btn" onmousedown="sendCmd('backward_left')" onmouseup="sendCmd('stop')">↙</button>
                        <button class="btn" onmousedown="sendCmd('backward')" onmouseup="sendCmd('stop')">↓<br>후진</button>
                        <button class="btn" onmousedown="sendCmd('backward_right')" onmouseup="sendCmd('stop')">↘</button>
                        
                        <button class="btn" onmousedown="sendCmd('rotate_left')" onmouseup="sendCmd('stop')">↶<br>좌회전</button>
                        <button class="btn"></button>
                        <button class="btn" onmousedown="sendCmd('rotate_right')" onmouseup="sendCmd('stop')">↷<br>우회전</button>
                    </div>
                </div>
                
                <!-- 탭 2: 자동 반복 -->
                <div class="tab-content" id="tab1">
                    <h2>자동 반복 설정</h2>
                    <div class="auto-repeat-control">
                        <label>동작 방향</label>
                        <select id="repeat-direction">
                            <option value="left_right">좌우</option>
                            <option value="forward_backward">전후</option>
                            <option value="rotation">회전</option>
                        </select>
                        
                        <label>이동 거리 (m)</label>
                        <input type="number" id="repeat-distance" value="1.0" step="0.1" min="0.1" max="10">
                        
                        <label>반복 횟수</label>
                        <input type="number" id="repeat-count" value="5" min="1" max="100">
                        
                        <label>대기 시간 (초)</label>
                        <input type="number" id="repeat-delay" value="1.0" step="0.5" min="0.5" max="10">
                        
                        <label>속도 (m/s)</label>
                        <input type="number" id="repeat-speed" value="0.2" step="0.05" min="0.1" max="0.5">
                    </div>
                    
                    <div class="progress-bar">
                        <div class="progress-fill" id="progress" style="width: 0%">0%</div>
                    </div>
                    <p id="repeat-status">대기 중...</p>
                    
                    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 15px;">
                        <button class="btn" onclick="startRepeat()">🚀 반복 시작</button>
                        <button class="btn btn-stop" onclick="stopRepeat()">⏹ 반복 중지</button>
                    </div>
                </div>
                
                <!-- 탭 3: 고급 설정 -->
                <div class="tab-content" id="tab2">
                    <h2>모터 제어</h2>
                    <button class="btn" onclick="pingMotors()" style="width: 100%; margin-bottom: 10px;">🔍 모터 Ping 테스트</button>
                    
                    <div style="display: grid; grid-template-columns: 1fr 2fr; gap: 10px; margin-bottom: 10px;">
                        <label>모터 ID:</label>
                        <input type="number" id="motor-id" value="1" min="1" max="10">
                    </div>
                    <button class="btn" onclick="initMotor()" style="width: 100%; margin-bottom: 15px;">🔧 모터 초기화</button>
                    
                    <h2>브레이크 제어</h2>
                    <select id="brake-mode" style="width: 100%; padding: 10px; margin-bottom: 10px;">
                        <option value="0">0 - Motor Brake</option>
                        <option value="1">1 - Motor Non-Brake</option>
                        <option value="2">2 - Outer Brake</option>
                        <option value="3">3 - Outer Non-Brake</option>
                        <option value="4">4 - Reset Pose</option>
                    </select>
                    <button class="btn" onclick="controlBrake()" style="width: 100%;">🛑 브레이크 실행</button>
                </div>
                
                <button class="btn btn-emergency" onclick="emergencyStop()">🚨 긴급 정지</button>
            </div>
            
            <!-- 모니터링 패널 -->
            <div class="monitor-section">
                <h2>로봇 상태</h2>
                <div class="status-grid">
                    <div class="status-item">
                        <div class="status-label">위치 X</div>
                        <div class="status-value" id="pos-x">0.00 m</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label">위치 Y</div>
                        <div class="status-value" id="pos-y">0.00 m</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label">방향</div>
                        <div class="status-value" id="theta">0.00°</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label">최소 거리</div>
                        <div class="status-value" id="min-dist">- m</div>
                    </div>
                </div>
                
                <h2>현재 명령</h2>
                <div class="status-grid">
                    <div class="status-item">
                        <div class="status-label">선속도</div>
                        <div class="status-value" id="cmd-linear">0.00 m/s</div>
                    </div>
                    <div class="status-item">
                        <div class="status-label">각속도</div>
                        <div class="status-value" id="cmd-angular">0.00 rad/s</div>
                    </div>
                </div>
                
                <h2>로그</h2>
                <div class="log-box" id="log-box"></div>
                <button class="btn" onclick="clearLog()" style="width: 100%; margin-top: 10px;">로그 지우기</button>
            </div>
        </div>
    </div>
    
    <script>
        // 탭 전환
        function switchTab(index) {
            document.querySelectorAll('.tab').forEach((tab, i) => {
                tab.classList.toggle('active', i === index);
            });
            document.querySelectorAll('.tab-content').forEach((content, i) => {
                content.classList.toggle('active', i === index);
            });
        }
        
        // 속도 업데이트
        function updateSpeed() {
            const linear = (document.getElementById('linear-speed').value / 100).toFixed(2);
            const angular = (document.getElementById('angular-speed').value / 100).toFixed(2);
            document.getElementById('linear-speed-val').textContent = linear;
            document.getElementById('angular-speed-val').textContent = angular;
        }
        
        // 명령 전송
        function sendCmd(direction) {
            const linear = document.getElementById('linear-speed').value / 100;
            const angular = document.getElementById('angular-speed').value / 100;
            
            fetch('/cmd', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({direction, linear, angular})
            });
        }
        
        // 긴급 정지
        function emergencyStop() {
            fetch('/emergency_stop', {method: 'POST'});
        }
        
        // 자동 반복
        function startRepeat() {
            const config = {
                direction: document.getElementById('repeat-direction').value,
                distance: parseFloat(document.getElementById('repeat-distance').value),
                count: parseInt(document.getElementById('repeat-count').value),
                delay: parseFloat(document.getElementById('repeat-delay').value),
                speed: parseFloat(document.getElementById('repeat-speed').value)
            };
            
            fetch('/start_repeat', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(config)
            });
        }
        
        function stopRepeat() {
            fetch('/stop_repeat', {method: 'POST'});
        }
        
        // 모터 제어
        function pingMotors() {
            fetch('/ping').then(r => r.json()).then(data => {
                console.log('Ping result:', data);
            });
        }
        
        function initMotor() {
            const motorId = document.getElementById('motor-id').value;
            fetch('/motor_init?motor_id=' + motorId, {method: 'POST'});
        }
        
        function controlBrake() {
            const mode = document.getElementById('brake-mode').value;
            fetch('/brake?mode=' + mode, {method: 'POST'});
        }
        
        // 로그 지우기
        function clearLog() {
            fetch('/clear_log', {method: 'POST'});
        }
        
        // 상태 업데이트 (1초마다)
        setInterval(() => {
            fetch('/state').then(r => r.json()).then(data => {
                document.getElementById('pos-x').textContent = data.pos_x.toFixed(2) + ' m';
                document.getElementById('pos-y').textContent = data.pos_y.toFixed(2) + ' m';
                document.getElementById('theta').textContent = (data.theta * 180 / Math.PI).toFixed(2) + '°';
                document.getElementById('min-dist').textContent = data.min_dist.toFixed(2) + ' m';
                document.getElementById('cmd-linear').textContent = data.current_linear.toFixed(2) + ' m/s';
                document.getElementById('cmd-angular').textContent = data.current_angular.toFixed(2) + ' rad/s';
                
                // 반복 진행률
                document.getElementById('progress').style.width = data.repeat_progress + '%';
                document.getElementById('progress').textContent = data.repeat_progress + '%';
                
                // 로그
                const logBox = document.getElementById('log-box');
                logBox.innerHTML = data.logs.slice(-20).join('<br>');
                logBox.scrollTop = logBox.scrollHeight;
            });
        }, 1000);
    </script>
</body>
</html>
'''

@app.route('/')
def index():
    """메인 페이지"""
    return render_template_string(HTML_TEMPLATE)

@app.route('/state')
def get_state():
    """로봇 상태 반환"""
    return jsonify(robot_state)

@app.route('/cmd', methods=['POST'])
def send_command():
    """방향 명령 전송"""
    data = request.json
    direction = data.get('direction')
    linear = data.get('linear', 0.2)
    angular = data.get('angular', 0.2)
    
    twist = Twist()
    
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
    
    cmd_vel_pub.publish(twist)
    robot_state['current_linear'] = twist.linear.x
    robot_state['current_angular'] = twist.angular.z
    
    if direction != "stop":
        log_message(f"🎮 {direction} (선속도: {twist.linear.x:.2f}, 각속도: {twist.angular.z:.2f})")
    
    return jsonify({'status': 'ok'})

@app.route('/emergency_stop', methods=['POST'])
def emergency_stop():
    """긴급 정지"""
    twist = Twist()
    cmd_vel_pub.publish(twist)
    robot_state['current_linear'] = 0.0
    robot_state['current_angular'] = 0.0
    log_message("🚨 긴급 정지!")
    return jsonify({'status': 'ok'})

@app.route('/start_repeat', methods=['POST'])
def start_repeat():
    """자동 반복 시작"""
    config = request.json
    log_message(f"🚀 자동 반복 시작: {config}")
    robot_state['auto_repeat_running'] = True
    # TODO: 실제 반복 로직 구현
    return jsonify({'status': 'ok'})

@app.route('/stop_repeat', methods=['POST'])
def stop_repeat():
    """자동 반복 중지"""
    robot_state['auto_repeat_running'] = False
    twist = Twist()
    cmd_vel_pub.publish(twist)
    log_message("⏹ 자동 반복 중지")
    return jsonify({'status': 'ok'})

@app.route('/ping', methods=['GET'])
def ping():
    """모터 Ping"""
    try:
        req = TriggerRequest()
        response = ping_service(req)
        results = response.message.split(",")
        log_message(f"🔍 Ping 결과: {results}")
        return jsonify({'results': results})
    except Exception as e:
        log_message(f"❌ Ping 실패: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/motor_init', methods=['POST'])
def motor_init():
    """모터 초기화"""
    motor_id = int(request.args.get('motor_id', 1))
    try:
        if CUSTOM_SRV_AVAILABLE:
            req = TriggerWithCommandRequest()
            req.command = motor_id - 1
            response = motor_init_service(req)
            log_message(f"🔧 모터 {motor_id} 초기화: {response.success}")
            return jsonify({'success': response.success})
        else:
            log_message("❌ 모터 초기화 서비스 사용 불가")
            return jsonify({'error': 'Service not available'}), 503
    except Exception as e:
        log_message(f"❌ 모터 초기화 실패: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/brake', methods=['POST'])
def brake():
    """브레이크 제어"""
    mode = int(request.args.get('mode', 0))
    try:
        if CUSTOM_SRV_AVAILABLE:
            req = TriggerWithCommandRequest()
            req.command = mode
            response = brake_service(req)
            log_message(f"🛑 브레이크 {mode}: {response.success}")
            return jsonify({'success': response.success})
        else:
            log_message("❌ 브레이크 서비스 사용 불가")
            return jsonify({'error': 'Service not available'}), 503
    except Exception as e:
        log_message(f"❌ 브레이크 실패: {e}")
        return jsonify({'error': str(e)}), 500

@app.route('/clear_log', methods=['POST'])
def clear_log():
    """로그 지우기"""
    robot_state['logs'].clear()
    return jsonify({'status': 'ok'})

def init_ros():
    """ROS 초기화"""
    global cmd_vel_pub, stop_mode_pub, control_mode_pub
    global ping_service, brake_service, motor_init_service
    
    # 퍼블리셔
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    stop_mode_pub = rospy.Publisher('/stop_mode', Bool, queue_size=1)
    control_mode_pub = rospy.Publisher('/control_mode', Bool, queue_size=1)
    
    # 서비스
    try:
        rospy.wait_for_service('ping_service', timeout=2.0)
        ping_service = rospy.ServiceProxy('ping_service', Trigger)
        
        if CUSTOM_SRV_AVAILABLE:
            brake_service = rospy.ServiceProxy('/brake_service', TriggerWithCommand)
            motor_init_service = rospy.ServiceProxy('/motor_init_service', TriggerWithCommand)
        
        log_message("✅ ROS 서비스 연결 성공")
    except rospy.ROSException as e:
        log_message(f"⚠️  ROS 서비스 연결 실패: {e}")
    
    # ROS 스레드 시작
    ros_thread = Thread(target=ros_thread_func, daemon=True)
    ros_thread.start()

def main():
    """메인 함수"""
    print("""
╔═══════════════════════════════════════════════════════════╗
║                                                           ║
║          Happy Robot Web Control UI                      ║
║          웹 브라우저 기반 제어 인터페이스                  ║
║                                                           ║
╚═══════════════════════════════════════════════════════════╝
    """)
    
    # ROS 초기화
    init_ros()
    
    # Flask 서버 시작
    print("\n🌐 웹 서버 시작...")
    print(f"🔗 브라우저에서 접속: http://localhost:5000")
    print(f"🔗 Jetson Nano IP: http://<JETSON_IP>:5000\n")
    
    app.run(host='0.0.0.0', port=5000, debug=False)

if __name__ == '__main__':
    main()
