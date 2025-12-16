#!/usr/bin/env python3.8

from fastapi import FastAPI, Query, Body
from fastapi.responses import Response, JSONResponse
import uvicorn
import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerRequest  # std_srvs/Trigger 사용
from happy_robo.srv import TriggerWithCommand, TriggerWithCommandRequest

rospy.init_node("teleop_receiver")
rospy.loginfo("teleop_receiver starting...")
app = FastAPI()

# 기존 퍼블리셔
init_topic = '/api/sector'
init_pub = rospy.Publisher(init_topic, Int32, queue_size=1)
park_topic = "/api/park"
park_pub = rospy.Publisher(park_topic, Bool, queue_size=1)
obb_topic = "/api/obb"
obb_pub = rospy.Publisher(obb_topic, Bool, queue_size=1)
control_topic = "/control_mode"
control_pub = rospy.Publisher(control_topic, Bool, queue_size=1)
stop_topic = "/stop_mode"
stop_pub = rospy.Publisher(stop_topic, Bool, queue_size=1)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# ROS 서비스 클라이언트 초기화
try :
    rospy.loginfo("wait_for_ping_service...")
    # rospy.wait_for_service('ping_service')
    ping_service = rospy.ServiceProxy('ping_service', Trigger)
    brake_service = rospy.ServiceProxy('/brake_service', TriggerWithCommand)
    motor_init_service = rospy.ServiceProxy('/motor_init_service', TriggerWithCommand)
    tank_service = rospy.ServiceProxy('/tank_service', TriggerWithCommand)
except rospy.ServiceException as e:
    rospy.logerr(f"Service call failed: {e}")

# 기존 API 엔드포인트 (생략)
@app.post("/init")
async def init_api(sector: int = Query(..., description="input init sector number")):
    init_pub.publish(sector)
    rospy.loginfo(f"✅ publish {sector} on Init Sector Decision API")
    return JSONResponse(content=f"✅ publish {sector} on Init Sector Decision API", status_code=200)

@app.post("/park")
async def park_api(complete: int = Query(..., description="input parking sequnece result")):
    msg = Bool()
    msg.data = True
    park_pub.publish(msg)
    rospy.loginfo(f"✅ publish {msg} on Parking Complete API")
    return JSONResponse(content=f"✅ publish Parking Complete Signal on Parking API", status_code=200)

@app.post("/obb")
async def park_api(approval: int = Query(..., description="input obb approval result")):
    msg = Bool()
    if (approval == 1):
        msg.data = True
    else : msg.data = False
    obb_pub.publish(msg)
    rospy.loginfo(f"✅ publish {msg} on OBB Approval API")
    return JSONResponse(content=f"✅ publish OBB Approval Signal on OBB API", status_code=200)

@app.post("/mode/control")
async def control_api(control: int = Query(..., description="input control number (1 : Vertical , 0 : Horizontal)")):
    msg = Bool()
    if control == 1:
        msg.data = True
        control_pub.publish(msg)
        rospy.loginfo(f"✅ publish {msg}, Vertical Mode")
    elif control == 0:
        msg.data = False
        control_pub.publish(msg)
        rospy.loginfo(f"✅ publish {msg}, Horizontal Mode")
    else:
        rospy.loginfo(f"Input 0 or 1 not {control}")
    return {"received_control_number": control}

@app.post("/mode/stop")
async def stop_api(stop: int = Query(..., description="input stop number (1 : Stop, 0 : Continue)")):
    msg = Bool()
    if stop == 1:
        msg.data = True
        stop_pub.publish(msg)
        rospy.loginfo(f"✅ publish {msg}, Stop Mode")
    elif stop == 0:
        msg.data = False
        stop_pub.publish(msg)
        rospy.loginfo(f"✅ publish {msg}, Non Stop Mode")
    else:
        rospy.loginfo(f"Input 0 or 1 not {stop}")
    return {"received_stop_number": stop}

@app.post("/move")
async def move_api(
    direction: str = Body(...),
    linear: float = Body(...),
    angular: float = Body(...),
    mode: str = Body(...),
    degree: float = Body(...)  # tank 각도 추가
):
    try:
        if direction in ["tank_up", "tank_down"]:
            req = TriggerWithCommandRequest()
            req.command = 0 if direction == "tank_up" else 1  # tank_up: 0, tank_down: 1
            req.degree = degree  # Flutter에서 받은 tank 각도 전달
            response = tank_service(req)
            result = response.message.strip() == "True"
            status_code = 200 if result else 503
            return JSONResponse(content={"result": result}, status_code=status_code)
        else:
            twist = Twist()
            linear = round(linear, 1)
            angular = round(angular, 1)
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
            elif direction == "stop":
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
            elif direction == "rotate_left":
                twist.angular.z = angular
            elif direction == "rotate_right":
                twist.angular.z = -angular
            rospy.loginfo(f"✅ Get Twist CMD : linear={twist.linear.x}, angular={twist.angular.z}")
            cmd_vel_pub.publish(twist)
            rospy.loginfo(f"✅ Published Twist: linear={twist.linear.x}, angular={twist.angular.z}")
            return {"direction": direction, "linear": linear, "angular": angular, "mode": mode}
    except rospy.ServiceException as e:
        return JSONResponse(content={"error": f"Move failed: {str(e)}"}, status_code=503)

# 핑 테스트 API (ROS 서비스 호출)
@app.get("/ping")
async def ping_api():
    try:
        req = TriggerRequest()
        response = ping_service(req)
        ping_results = [res.strip() == "True" for res in response.message.split(",")]  # bool 리스트
        result_dict = {f"id{i+1}": bool(res) for i, res in enumerate(ping_results)}  # 명시적 bool 변환
        status_code = 503 if False in ping_results else 200
        return JSONResponse(content=result_dict, status_code=status_code)
    except rospy.ServiceException as e:
        return JSONResponse(content={"error": f"Ping failed: {str(e)}"}, status_code=503)
    
@app.get("/brake")
async def brake_api(command: int = Query(..., ge=0, le=4)):  # 0~4 범위 제한
    try:
        req = TriggerWithCommandRequest()
        req.command = command  # 커스텀 메시지의 command 필드에 값 설정
        response = brake_service(req)
        result = response.message.strip() == "True"
        result_dict = {"result": result}
        status_code = 200 if result else 503
        return JSONResponse(content=result_dict, status_code=status_code)
    except rospy.ServiceException as e:
        return JSONResponse(content={"error": f"Brake failed: {str(e)}"}, status_code=503)
    
@app.get("/motor_init")
async def motor_init_api(command: int = Query(..., ge=1, le=10)):  # 1~10 범위 제한
    try:
        req = TriggerWithCommandRequest()
        req.command = command - 1  # Flutter에서 1~10를 보내지만 C++에서 0~9 인덱스로 처리
        response = motor_init_service(req)
        result = response.message.strip() == "True"
        result_dict = {"result": result}
        status_code = 200 if result else 503
        return JSONResponse(content=result_dict, status_code=status_code)
    except rospy.ServiceException as e:
        return JSONResponse(content={"error": f"Motor init failed: {str(e)}"}, status_code=503)

@app.get("/")
def root():
    return {"Happy Robot Autonomous Vehicle Remote Controller"}

if __name__ == "__main__":
    try:
        uvicorn.run(app, host="0.0.0.0", port=8000)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")