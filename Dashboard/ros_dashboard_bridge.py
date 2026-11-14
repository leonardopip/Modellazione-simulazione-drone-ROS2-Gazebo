
#!/usr/bin/env python3
"""Ponte ROS 2 Jazzy -> dashboard drone (WebSocket, 10 Hz).
 
Legge: /x3/odometry, /x3/imu, /X3/gazebo/command/motor_speed, /joy
Installazione:  sudo apt install python3-websockets
Avvio:          python3 ros_dashboard_bridge.py   (tieni drone-dashboard.html nella stessa cartella)
Poi apri:       http://localhost:8000
Parametri (-p nome:=valore): odom_topic, imu_topic, motor_topic, joy_topic,
                             deadzone (default 0.1), port (default 8765)
"""
import asyncio
import functools
import http.server
import json
import math
import os
import threading
import time
 
import rclpy
import websockets
from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, Joy
 
JOY_TIMEOUT = 1.0  # secondi senza messaggi dal pad -> "Attesa pad"
 
 
class Bridge(Node):
    def __init__(self):
        super().__init__("dashboard_bridge")
        for name, default in [
            ("odom_topic", "/x3/odometry"),
            ("imu_topic", "/x3/imu"),
            ("motor_topic", "/X3/gazebo/command/motor_speed"),
            ("joy_topic", "/joy"),
            ("deadzone", 0.1),
            ("port", 8765),
            ("http_port", 8000),
        ]:
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value
 
        self.joy_time = 0.0
        self.odom_time = None
        self.armed_latch = False   # accensione: resta ON dopo il fronte di salita
        self.prev_btn = (0, 0)
        self.state = {
            "x": 0.0, "y": 0.0, "alt": 0.0, "spd": 0.0,
            "roll": 0.0, "pitch": 0.0, "hdg": 0.0,
            "gyro": [0.0, 0.0, 0.0], "acc": [0.0, 0.0, 0.0],
            "motors": [0.0, 0.0, 0.0, 0.0],
            "cmd": {"thr": 0.0, "pitch": 0.0, "roll": 0.0,
                    "rotSx": 0, "rotDx": 0, "arm": 0, "off": 0},
            "mode": "Attesa pad", "armed": False,
            "batt": 100, "sats": 0,  # non disponibili in simulazione: valori fissi
        }
        self.create_subscription(Odometry, p("odom_topic"), self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Imu, p("imu_topic"), self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Actuators, p("motor_topic"), self.on_motors, 10)
        self.create_subscription(Joy, p("joy_topic"), self.on_joy, 10)
        self.create_timer(3.0, self.check_odom)
 
    def check_odom(self):
        if self.odom_time is None:
            self.get_logger().warn(
                "Nessun messaggio sull'odometria: Gazebo e' in pausa o il topic e' sbagliato?")
 
    def on_odom(self, m):
        self.odom_time = time.time()
        p, q, v = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.linear
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        self.state.update({
            "x": p.x, "y": p.y, "alt": p.z,              # ENU: x est, y nord
            "spd": math.hypot(v.x, v.y) * 3.6,           # m/s -> km/h
            "roll": math.degrees(roll),
            "pitch": math.degrees(pitch),
            "hdg": (90 - math.degrees(yaw)) % 360,       # yaw ENU -> rotta dal nord
        })
 
    def on_imu(self, m):
        g, a = m.angular_velocity, m.linear_acceleration
        self.state["gyro"] = [math.degrees(g.x), math.degrees(g.y), math.degrees(g.z)]
        self.state["acc"] = [a.x, a.y, a.z]
 
    def on_motors(self, m):
        speeds = [float(v) for v in m.velocity]
        self.state["motors"] = speeds
        self.state["armed"] = any(v > 0 for v in speeds)
 
    def on_joy(self, m):
        dz = self.get_parameter("deadzone").value
        d = lambda v: 0.0 if abs(v) < dz else v
        c = self.state["cmd"]
        # stessa mappatura del tuo Pad_Sub (valori grezzi, senza filtro passa-basso)
        if len(m.axes) > 4:
            c["thr"] = d(m.axes[4])
            c["pitch"] = d(m.axes[1])
            c["roll"] = d(-m.axes[0])
        if len(m.buttons) > 5:
            c["rotDx"], c["rotSx"] = int(m.buttons[5]), int(m.buttons[4])
            arm, off = int(m.buttons[0]), int(m.buttons[1])
            if arm and not self.prev_btn[0]:
                self.armed_latch = True    # fronte di salita: accensione ON e resta ON
            if off and not self.prev_btn[1]:
                self.armed_latch = False   # fronte di salita: spegnimento
            self.prev_btn = (arm, off)
            c["arm"], c["off"] = int(self.armed_latch), off
        self.joy_time = time.time()
 
 
def clean(v):
    """NaN/inf non sono JSON valido: la dashboard scarterebbe tutto il messaggio."""
    if isinstance(v, float) and not math.isfinite(v):
        return 0.0
    if isinstance(v, dict):
        return {k: clean(x) for k, x in v.items()}
    if isinstance(v, list):
        return [clean(x) for x in v]
    return v
 
 
def serve_http(port):
    """Serve drone-dashboard.html (nella stessa cartella dello script)."""
    here = os.path.dirname(os.path.abspath(__file__))
 
    class Handler(http.server.SimpleHTTPRequestHandler):
        def do_GET(self):
            if self.path in ("", "/"):
                self.path = "/drone-dashboard.html"
            super().do_GET()
 
        def log_message(self, *args):
            pass
 
    handler = functools.partial(Handler, directory=here)
    http.server.ThreadingHTTPServer(("0.0.0.0", port), handler).serve_forever()
 
 
async def serve(node):
    clients = set()
 
    async def handler(ws):
        clients.add(ws)
        try:
            await ws.wait_closed()
        finally:
            clients.discard(ws)
 
    async with websockets.serve(handler, "0.0.0.0", node.get_parameter("port").value):
        while True:
            fresh = time.time() - node.joy_time < JOY_TIMEOUT
            node.state["mode"] = "Manuale" if fresh else "Attesa pad"
            node.state["armed"] = node.armed_latch or any(v > 0 for v in node.state["motors"])
            msg = json.dumps(clean(node.state))
            for c in list(clients):
                try:
                    await c.send(msg)
                except Exception:
                    clients.discard(c)
            await asyncio.sleep(0.1)  # 10 Hz
 
 
def main():
    rclpy.init()
    node = Bridge()
    threading.Thread(target=lambda: asyncio.run(serve(node)), daemon=True).start()
    http_port = node.get_parameter("http_port").value
    threading.Thread(target=serve_http, args=(http_port,), daemon=True).start()
    node.get_logger().info(f"Dashboard: http://localhost:{http_port}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
 
