from enum import Enum
from flask import Flask, Response, render_template
import cv2
import json
import numpy as np
import time
import paho.mqtt.client as mqtt
from dataclasses import dataclass

class RobotState(Enum):
    OFFLINE = 0
    IDLE = 1
    ACTIVE_MOVING = 2
    ACTIVE_STOPPED = 3
    OBSTRUCTED_STOPPED = 4
    OBSTRUCTED_REROUTING = 5
    OBSTRUCTED_SCANNING = 6

# thing for the robot
# robot class from the perspective of the server
# class Robot:
#     ip = "12.34.567.89"
#     battery = 1.0
#     state = RobotState.OFFLINE
#     globe_dia = 12
#     robot_number = 1

#     def __init__(self, ip, battery, state, globe_dia, robot_number):
#         # takes a string IP,
#         # a double representing the percent battery level remaining,
#         # a RobotState representing the current RobotState,
#         # an int representing the diameter of the globe, in inches
#         # and the robot's number (1-17)
#         self.ip = ip
#         self.battery = battery
#         self.state = state
#         self.globe_dia = globe_dia
#         self.robot_number = robot_number
#         pass

#     def printInfo(self):
#         return f"Robot #{self.robot_number}\nIP: {self.ip}\nBattery: {self.battery * 100}%\nState: {self.state}\nDiameter: {self.globe_dia}"


BROKER = "midway.breckstampede.org"  # Change to your broker address
TOPIC = "robots/#"  # Subscribe to all subtopics
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(TOPIC)
    else:
        print(f"Failed to connect, return code {reason_code}")

robots = {}
def on_message(client, userdata, msg):
    topics = msg.topic.split('/')
    if topics[0] == "robots" and not topics[1] in robots:
        robots[topics[1]] = (Robot(name=topics[1]))
        print(f"Robot {topics[1]} connected")

    if topics[2] == "imu" and topics[3] == "euler":
     # read json from msg.payload
        data = json.loads(msg.payload.decode())
        heading = RobotHeading(
            x=data['x'],
            y=data['y'],
            z=data['z'],
            rad_accuracy=data['rad_accuracy'],
            accuracy=data['accuracy']
        )
        robots[topics[1]].heading = heading
        # print(f"Robot {topics[1]} heading: {heading}")

    if topics[2] == "led":
        # read json from msg.payload
        data = json.loads(msg.payload.decode())
        led = LED(
            color = Color(
                r=data['r'],
                g=data['g'],
                b=data['b']
            ),
            step=data['step']
        )
        robots[topics[1]].led = led

    if topics[3] == "telemetry":
        data = json.loads(msg.payload.decode())
        telemetry = Telemetry(
            speed=data['speed'],
            ticks=data['ticks'],
            output=data['output'],
            setpoint=data['setpoint'],
            hasPower=data['hasPower']
            # json in c++ might need to be changed to True and False (instead of true and false)
        )
        robots[topics[1]].telemetry = telemetry
    
    if topics[2] == "odometry":
        if topics[3] == "pose":
            data = json.loads(msg.payload.decode())
            pose = Pose(
                x=data['x'],
                y=data['y'],
                heading=data['heading']
            )
            robots[topics[1]].pose = pose

        elif topics[3] == "pose2":
            data = json.loads(msg.payload.decode())
            pose2 = Pose(
                x=data['x'],
                y=data['y'],
                heading=data['heading']
            )
            robots[topics[1]].pose2 = pose2

    if topics[2] == "wifi" and topics[3] == "rssi":
        data = json.loads(msg.payload.decode())
        rssi = data['rssi']
        robots[topics[1]].wifi_rssi = rssi  

    if topics[2] == "tcs":
        data = json.loads(msg.payload.decode())
        status = data['status']
        if status == "ok":
            tcs = TCS(
                r=data['r'],
                g=data['g'],
                b=data['b'],
                c=data['c'],
                lux=data['lux'],
                color_temperature=data['color_temperature'],
                status=data['status']
            )
            robots[topics[1]].tcs = tcs
        else:
            tcs = TCS(
                r=0,
                g=0,
                b=0,
                c=0,
                lux=0,
                color_temperature=0,
                status=status
            )
            robots[topics[1]].tcs = tcs

            robots[topics[1]].tcs.color_temperature = 0
            robots[topics[1]].tcs.status = status
            robots[topics[1]].tcs = tcs
        
    if topics[2] == "reroute":
        data = json.loads(msg.payload.decode())
        robots[topics[1]].reroute = data['reroute']

    if msg.topic.endswith("/ping") and msg.payload.decode() == "pong":
        robots[topics[1]].last_ping = time.time()

@dataclass
class Color:
    r: int
    g: int
    b: int

    def __str__(self):
        return f"[{self.r}, {self.g}, {self.b}]"

@dataclass
class LED:
    color: Color
    step: int

@dataclass
class RobotHeading:
    x: float
    y: float
    z: float
    rad_accuracy: float
    accuracy: str

@dataclass
class Telemetry:
    speed: float
    ticks: int
    output: float
    setpoint: float
    hasPower: bool
    
@dataclass
class Pose:
    x: float
    y: float
    heading: float

@dataclass
class TCS:
    r: int
    g: int
    b: int
    c: int
    lux: int
    color_temperature: int
    status: str

@dataclass
class Robot:
    name: str
    last_ping: float = 0
    led = LED(Color(0, 0, 0), 0)
    heading = RobotHeading(0.0, 0.0, 0.0, 0.0, "UNDEFINED")
    telemetry = Telemetry(0.0, 0, 0.0, 0.0, False)
    pose = Pose(0.0, 0.0, 0.0)
    pose2 = Pose(0.0, 0.0, 0.0)
    wifi_rssi: int = 0
    tcs = TCS(0, 0, 0, 0, 0, 0, "not_initialized")
    reroute: bool
    blink_pattern: list[Color] = []
    x_pattern: list[float] = []
    y_pattern: list[float] = []

    def send_ping(self):
        client.publish(f"robots/{self.name}/ping", "ping")

    def send_blink_pattern(self):
        client.publish(f"robots/{self.name}/command/blink_pattern", ", ".join(str(color) for color in self.blink_pattern))

    def send_pattern(self):
        client.publish(f"robots/{self.name}/command/pattern", "[" + ", ".join(str(x) for x in self.x_pattern) + "], [" + ", ".join(str(y) for y in self.y_pattern) + "]")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
client.tls_set(ca_certs="ca.pem")
client.username_pw_set("esp", "esp32")
client.connect(BROKER, 8883, 60)
client.loop_start()
