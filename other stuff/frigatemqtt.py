import threading
import time
from dataclasses import dataclass
import paho.mqtt.client as mqtt

BROKER = "midway.breckstampede.org"  # Change to your broker address
TOPIC = "frigate/#"  # Subscribe to all subtopics
def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("Connected to MQTT Broker!")
        client.subscribe(TOPIC)
    else:
        print(f"Failed to connect, return code {reason_code}")

def on_message(client, userdata, msg):
    if (msg.topic == "frigate/events"):
        import json
        payload = msg.payload.decode('utf-8')
        # print(f"Received message on {msg.topic}: {payload}")
        try:
            data = json.loads(payload)
            after = data.get('after', {})
            label = after.get('label')
            score = after.get('score')
            camera = after.get('camera')
            box = after.get('box')
            if label == 'person':
                print(f"Human detected on camera '{camera}' at box {box} with confidence: {score*100:.1f}%")
            else:
                print(f"No human detected on camera '{camera}'.")
        except Exception as e:
            print(f"Error parsing JSON: {e}")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
client.tls_set(ca_certs="ca.pem")
client.username_pw_set("esp", "esp32")
client.connect(BROKER, 8883, 60)
client.loop_forever()



