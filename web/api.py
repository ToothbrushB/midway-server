from flask import Blueprint, request
from mqtt import client
api_bp = Blueprint('api_bp', __name__)

@api_bp.post('/mqtt')
def mqtt():
    topic = request.form['topic']
    payload = request.form['payload']
    if not topic or not payload:
        return "Topic and payload are required", 400
    client.publish(topic, payload)
    return "Message published", 200