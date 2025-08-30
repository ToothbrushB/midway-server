# MQTT stub for when MQTT is not available
class MockMQTTClient:
    def publish(self, topic, payload):
        print(f"MQTT (mock): {topic} -> {payload}")
        return True
    
    def connect(self, host, port):
        print(f"MQTT (mock): Connected to {host}:{port}")
        return True

client = MockMQTTClient()
