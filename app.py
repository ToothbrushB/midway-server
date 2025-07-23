from flask import Flask, Response, render_template
import cv2
import json
import numpy as np
import time
import paho.mqtt.client as mqtt
from dataclasses import dataclass



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
    
    if msg.topic.endswith("/ping") and msg.payload.decode() == "pong":
        robots[topics[1]].last_ping = time.time()


@dataclass
class Robot:
    name: str
    last_ping: float = 0

    def send_ping(self):
        client.publish(f"robots/{self.name}/ping", "ping")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
client.tls_set(ca_certs="ca.pem")
client.username_pw_set("esp", "esp32")
client.connect(BROKER, 8883, 60)
client.loop_start()

def load_calibration(filename):
    with open(filename, 'r') as f:
        calib = json.load(f)
    camera_matrix = np.array(calib['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    dist_coeffs = np.array(calib['distortion_coefficients']['data'], dtype=np.float64).reshape(-1, 1)
    return camera_matrix, dist_coeffs

camera_matrix, dist_coeffs = load_calibration('calib_logi_c310_hd_webcam__046d_081b__1280.json')

COLOR_RANGES = {
    'red': ([0, 100, 100], [10, 255, 255]),
    'green': ([40, 100, 100], [80, 255, 255]),
    'blue': ([100, 100, 100], [140, 255, 255])
}

cap = cv2.VideoCapture(0)
def gen_frames():
    prev_time = time.time()
    frame_count = 0
    fps = 0
    while True:
        success, frame = cap.read()
        if not success:
            break
        # Undistort the frame using the camera matrix and distortion coefficients
        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        for color, (lower, upper) in COLOR_RANGES.items():
            lower = np.array(lower, dtype=np.uint8)
            upper = np.array(upper, dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            # https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html#ga819779b9857cc2f8601e6526a3a5bc71 and https://docs.opencv.org/4.x/d4/d73/tutorial_py_contours_begin.html
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [c for c in contours if cv2.contourArea(c) > 100]
            cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)

        # FPS calculation
        frame_count += 1
        curr_time = time.time()
        if curr_time - prev_time >= 1.0:
            fps = frame_count / (curr_time - prev_time)
            prev_time = curr_time
            frame_count = 0
        # Draw FPS on frame
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')