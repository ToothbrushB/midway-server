import csv
import threading
import time

import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt

BROKER = "midway.breckstampede.org"  # Change to your broker address
TOPIC = "robots/#"  # Subscribe to all subtopics

i = 0
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(TOPIC)

    else:
        print(f"Failed to connect, return code {rc}")

def on_message(client, userdata, msg):
    global i
    if msg.topic.endswith("start"):
        on_message.start = True
    elif msg.topic.endswith("/odometry/pose") and hasattr(on_message, 'start') and on_message.start:
        # Pose data: x, y, heading
        try:
            x, y, heading = map(float, msg.payload.decode().split(','))
        except Exception as e:
            print(f"Error parsing pose: {e}")
            return
        # print(f"Received pose: x={x}, y={y}, heading={heading}")
        if not hasattr(on_message, 'pose_data'):
            on_message.pose_data = []
        on_message.pose_data.append((x, y, heading, int(msg.properties.UserProperty[0][1])/1e6))
    elif msg.topic.endswith("/odometry/pose2") and hasattr(on_message, 'start') and on_message.start:
        # Pose data: x, y, heading
        try:
            x, y, heading = map(float, msg.payload.decode().split(','))
        except Exception as e:
            print(f"Error parsing pose: {e}")
            return
        if not hasattr(on_message, 'pose_data2'):
            on_message.pose_data2 = []
        on_message.pose_data2.append((x, y, heading, int(msg.properties.UserProperty[0][1])/1e6))
    elif msg.topic.endswith("/path"):
        i += 1
        # Path data: x, y
        try:
            x, y = map(float, msg.payload.decode().split(','))
        except Exception as e:
            print(f"Error parsing path: {e}")
            return
        print(f"Received path: x={x}, y={y}, i={i}")
        # # Plot path
        # plt.scatter(x, y, color='blue', label='Path' if not hasattr(on_message, 'path_plotted') else "")
        # on_message.path_plotted = True
        # Store path data
        if not hasattr(on_message, 'path_data'):
            on_message.path_data = []
        on_message.path_data.append((x, y))


import matplotlib.pyplot as plt

# Function to plot when user presses a key
def plot_data():
    plt.figure()
    # Plot path
    path = None
    pose = None
    pose2= None
    # if hasattr(on_message, 'path_data'):
    #     path = on_message.path_data
    # if path:
    #     px, py = zip(*path)
    #     plt.scatter(px, py, color='blue', label='Path')
    # Plot pose
    if hasattr(on_message, 'pose_data'):
        pose = on_message.pose_data
    if pose:
        _, _, heading, time = zip(*pose)
        #plot heading over time
        plt.plot(time, heading, color='red', label='Heading')
    if hasattr(on_message, 'pose_data2'):
        pose2 = on_message.pose_data2
    if pose2:
        _, _, heading, time = zip(*pose2)
        plt.plot(time, heading, color='green', label='Heading2')
    plt.legend()
    plt.show()
    




client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, protocol=mqtt.MQTTv5)
client.on_connect = on_connect
client.on_message = on_message

client.tls_set(ca_certs="ca.pem")
client.username_pw_set("esp", "esp32")
client.connect(BROKER, 8883, 60)
client.loop_start()
# client.loop_forever()
while True:
    # wait = input("Press Enter to plot data...")
    plot_data()
    time.sleep(1)
    plt.close('all')