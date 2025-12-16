#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import json
import os
from typing import Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

import rospy
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerRequest
from happy_robo.srv import TriggerWithCommand, TriggerWithCommandRequest

rospy.init_node("teleop_ws_bridge", anonymous=True)
app = FastAPI(title="HappyRobo WebSocket Bridge")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_credentials=True, allow_methods=["*"], allow_headers=["*"])

# Publishers
init_pub = rospy.Publisher('/api/sector', Int32, queue_size=1)
park_pub = rospy.Publisher('/api/park', Bool, queue_size=1)
obb_pub = rospy.Publisher('/api/obb', Bool, queue_size=1)
control_pub = rospy.Publisher('/control_mode', Bool, queue_size=1)
stop_pub = rospy.Publisher('/stop_mode', Bool, queue_size=1)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# Services
try:
    ping_service = rospy.ServiceProxy('ping_service', Trigger)
    brake_service = rospy.ServiceProxy('/brake_service', TriggerWithCommand)
    motor_init_service = rospy.ServiceProxy('/motor_init_service', TriggerWithCommand)
    tank_service = rospy.ServiceProxy('/tank_service', TriggerWithCommand)
except Exception as e:
    rospy.logwarn(f"Service proxy creation error: {e}")
    ping_service = brake_service = motor_init_service = tank_service = None


class Hub:
    def __init__(self):
        self.peers: Set[WebSocket] = set()
        self.lock = asyncio.Lock()

    async def connect(self, ws: WebSocket):
        await ws.accept()
        async with self.lock:
            self.peers.add(ws)
        await ws.send_text(json.dumps({"type": "status", "status": "connected"}))

    async def disconnect(self, ws: WebSocket):
        async with self.lock:
            if ws in self.peers:
                self.peers.remove(ws)

    async def reply(self, ws: WebSocket, req: dict, **extra):
        resp = {"type": "result", "ok": True, **extra}
        if 'reqId' in req:
            resp['reqId'] = req['reqId']
        await ws.send_text(json.dumps(resp, ensure_ascii=False))

    async def error(self, ws: WebSocket, req: dict, message: str):
        resp = {"type": "result", "ok": False, "error": message}
        if 'reqId' in req:
            resp['reqId'] = req['reqId']
        await ws.send_text(json.dumps(resp, ensure_ascii=False))


hub = Hub()


async def handle_action(ws: WebSocket, req: dict):
    action = req.get("action")
    try:
        if action == "init":
            sector = int(req.get("sector"))
            init_pub.publish(sector)
            rospy.loginfo(f"✅ [init] sector={sector}")
            await hub.reply(ws, req, action=action)

        elif action == "park":
            msg = Bool(data=True)
            park_pub.publish(msg)
            rospy.loginfo(f"✅ [park] published True")
            await hub.reply(ws, req, action=action)

        elif action == "obb":
            approval = int(req.get("approval", 0))
            msg = Bool(data=True if approval == 1 else False)
            obb_pub.publish(msg)
            rospy.loginfo(f"✅ [obb] approval={approval}")
            await hub.reply(ws, req, action=action)

        elif action == "mode_control":
            control = int(req.get("control"))
            msg = Bool(data=True if control == 1 else False)
            control_pub.publish(msg)
            rospy.loginfo(f"✅ [mode_control] control={control}")
            await hub.reply(ws, req, action=action)

        elif action == "mode_stop":
            stop = int(req.get("stop"))
            msg = Bool(data=True if stop == 1 else False)
            stop_pub.publish(msg)
            rospy.loginfo(f"✅ [mode_stop] stop={stop}")
            await hub.reply(ws, req, action=action)

        elif action == "move":
            direction = req.get("direction")
            linear = float(req.get("linear", 0.0))
            angular = float(req.get("angular", 0.0))
            degree = float(req.get("degree", 0.0))

            if direction in ["tank_up", "tank_down"] and tank_service is not None:
                sreq = TriggerWithCommandRequest()
                sreq.command = 0 if direction == "tank_up" else 1
                sreq.degree = degree
                resp = tank_service(sreq)
                ok = (resp.message.strip() == "True")
                rospy.loginfo(f"✅ [tank_service] direction={direction}, degree={degree}, resp={resp.message}, ok={ok}")
                await hub.reply(ws, req, action=action, result=ok)
            else:
                tw = Twist()
                linear = round(linear, 1)
                angular = round(angular, 1)

                if direction == "forward":
                    tw.linear.x = linear
                elif direction == "backward":
                    tw.linear.x = -linear
                elif direction == "left":
                    tw.linear.y = linear
                elif direction == "right":
                    tw.linear.y = -linear
                elif direction == "forward_left":
                    tw.linear.x = linear; tw.angular.z = angular
                elif direction == "forward_right":
                    tw.linear.x = linear; tw.angular.z = -angular
                elif direction == "backward_left":
                    tw.linear.x = -linear; tw.angular.z = -angular
                elif direction == "backward_right":
                    tw.linear.x = -linear; tw.angular.z = angular
                elif direction == "stop":
                    tw.linear.x = 0.0; tw.linear.y = 0.0; tw.angular.z = 0.0
                elif direction == "rotate_left":
                    tw.angular.z = angular
                elif direction == "rotate_right":
                    tw.angular.z = -angular

                cmd_vel_pub.publish(tw)
                rospy.loginfo(f"✅ [move] direction={direction}, linear={linear}, angular={angular}")
                await hub.reply(ws, req, action=action)

        elif action == "ping":
            if ping_service is None:
                await hub.error(ws, req, "ping_service unavailable")
                return
            sreq = TriggerRequest()
            resp = ping_service(sreq)
            vals = [r.strip() == "True" for r in resp.message.split(",")]
            data = {f"id{i+1}": bool(v) for i, v in enumerate(vals)}
            rospy.loginfo(f"✅ [ping_service] resp={resp.message}, parsed={data}")
            await hub.reply(ws, req, action=action, data=data, ok=all(vals))

        elif action == "brake":
            if brake_service is None:
                await hub.error(ws, req, "brake_service unavailable")
                return
            c = int(req.get("command", 0))
            sreq = TriggerWithCommandRequest()
            sreq.command = c
            resp = brake_service(sreq)
            ok = (resp.message.strip() == "True")
            rospy.loginfo(f"✅ [brake_service] command={c}, resp={resp.message}, ok={ok}")
            await hub.reply(ws, req, action=action, result=ok, ok=ok)

        elif action == "motor_init":
            if motor_init_service is None:
                await hub.error(ws, req, "motor_init_service unavailable")
                return
            c = int(req.get("command", 1)) - 1
            sreq = TriggerWithCommandRequest()
            sreq.command = c
            resp = motor_init_service(sreq)
            ok = (resp.message.strip() == "True")
            rospy.loginfo(f"✅ [motor_init_service] command={c}, resp={resp.message}, ok={ok}")
            await hub.reply(ws, req, action=action, result=ok, ok=ok)

        else:
            await hub.error(ws, req, f"unknown action: {action}")

    except Exception as e:
        rospy.loginfo(f"❌ [error] action={action}, error={e}")
        await hub.error(ws, req, str(e))


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await hub.connect(ws)
    try:
        while True:
            try:
                raw = await ws.receive_text()
                try:
                    req = json.loads(raw)
                except Exception:
                    await ws.send_text(json.dumps({"type": "error", "error": "invalid json"}))
                    continue

                if isinstance(req, dict) and req.get("type") == "ping":
                    rospy.loginfo("✅ ping - pong")
                    await ws.send_text(json.dumps({"type": "pong"}))
                    continue

                await handle_action(ws, req)
            except asyncio.TimeoutError:
                await ws.send_text(json.dumps({"type": "server_heartbeat"}))
    except WebSocketDisconnect:
        pass
    finally:
        await hub.disconnect(ws)


if __name__ == "__main__":
    host = os.getenv("WS_HOST", "0.0.0.0")
    port = int(os.getenv("WS_PORT", "8000"))
    uvicorn.run(app, host=host, port=port, reload=False, log_level="info")
