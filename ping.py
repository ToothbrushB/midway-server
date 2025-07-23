import threading
import time
from dataclasses import dataclass
import paho.mqtt.client as mqtt



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

while True:
    robots_to_remove = []
    for robot in robots.values():
        robot.send_ping()
        if not robot.last_ping == 0 and time.time() - robot.last_ping > 5:
            print(f"Robot {robot.name} is offline")
            robots_to_remove.append(robot.name)
    
    # Remove offline robots after iteration
    for robot_name in robots_to_remove:
        robots.pop(robot_name, None)
    
    time.sleep(2)