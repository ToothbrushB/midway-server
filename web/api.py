from flask import Blueprint, request
from mqtt import client
import requests
from dotenv import load_dotenv
import os
import json

# Load environment variables from .env file
load_dotenv()

api_bp = Blueprint('api_bp', __name__)

@api_bp.post('/mqtt')
def mqtt():
    topic = request.form['topic']
    payload = request.form['payload']
    if not topic or not payload:
        return "Topic and payload are required", 400
    client.publish(topic, payload)
    return "Message published", 200

@api_bp.post('/notification')
def notification():
    title = request.form['title']
    content = request.form['content']
    if not title or not content:
        return "Title and content are required", 400
    r = requests.post('https://api.onesignal.com/notifications', 
                     headers={'Authorization': f"key {os.getenv('ONESIGNAL_API_KEY')}"}, json={
                         "app_id": os.getenv('ONESIGNAL_APP_ID'),
                         "headings": {"en": title},
                         "contents": {"en": content},
                        "included_segments": ["Test Users"]
                     })
    if r.status_code == 200 or r.status_code == 201:
        return "Notification sent", 200
    print(r.text)
    return "Failed to send notification", 500