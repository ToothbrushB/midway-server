from flask import Blueprint, request, jsonify, Response
try:
    from mqtt import client
except ImportError:
    print("MQTT not available, using mock client")
    from mqtt_stub import client
import requests
from dotenv import load_dotenv
import os
import json
import time
from datetime import datetime, timedelta
import cv2
import threading
import base64
import numpy as np
#RTSP://127.0.0.1:8554/cam_X


# Load environment variables from .env file
load_dotenv()

api_bp = Blueprint('api_bp', __name__)

# Camera management
active_cameras = {}
camera_threads = {}

class CameraStream:
    def __init__(self, camera_id, rtsp_url):
        self.camera_id = camera_id
        self.rtsp_url = rtsp_url
        self.cap = None
        self.frame = None
        self.running = False
        self.last_frame_time = time.time()
        
    def start(self):
        """Start the camera stream"""
        try:
            self.cap = cv2.VideoCapture(self.rtsp_url)
            if not self.cap.isOpened():
                print(f"Failed to open camera {self.camera_id} at {self.rtsp_url}")
                return False
            
            self.running = True
            thread = threading.Thread(target=self._update_frame)
            thread.daemon = True
            thread.start()
            return True
        except Exception as e:
            print(f"Error starting camera {self.camera_id}: {e}")
            return False
    
    def _update_frame(self):
        """Continuously update frames from the camera"""
        while self.running:
            try:
                ret, frame = self.cap.read()
                if ret:
                    self.frame = frame
                    self.last_frame_time = time.time()
                else:
                    time.sleep(0.1)  # Wait before retrying
            except Exception as e:
                print(f"Error reading frame from camera {self.camera_id}: {e}")
                time.sleep(1)
    
    def get_frame(self):
        """Get the latest frame as JPEG bytes"""
        if self.frame is not None:
            try:
                # Resize frame for web display (optional)
                height, width = self.frame.shape[:2]
                if width > 640:
                    scale = 640 / width
                    new_width = int(width * scale)
                    new_height = int(height * scale)
                    resized_frame = cv2.resize(self.frame, (new_width, new_height))
                else:
                    resized_frame = self.frame
                
                # Encode frame as JPEG
                ret, buffer = cv2.imencode('.jpg', resized_frame)
                if ret:
                    return buffer.tobytes()
            except Exception as e:
                print(f"Error encoding frame from camera {self.camera_id}: {e}")
        return None
    
    def get_frame_base64(self):
        """Get the latest frame as base64 encoded string"""
        frame_bytes = self.get_frame()
        if frame_bytes:
            return base64.b64encode(frame_bytes).decode('utf-8')
        return None
    
    def is_alive(self):
        """Check if camera is still receiving frames"""
        return self.running and (time.time() - self.last_frame_time) < 5.0
    
    def stop(self):
        """Stop the camera stream"""
        self.running = False
        if self.cap:
            self.cap.release()

def create_demo_camera(camera_id):
    """Create a demo camera stream for testing"""
    
    class DemoCamera:
        def __init__(self, camera_id):
            self.camera_id = camera_id
            self.running = True
            self.last_frame_time = time.time()
            self.frame_count = 0
            
        def start(self):
            return True
            
        def get_frame(self):
            # Generate a simple test pattern using OpenCV only
            self.frame_count += 1
            
            # Create a colored test pattern
            width, height = 640, 480
            
            # Background color based on camera ID
            colors = {
                '1': (50, 50, 150),   # Blue
                '2': (50, 150, 50),   # Green  
                '3': (150, 50, 50),   # Red
                '4': (150, 150, 50)   # Yellow
            }
            
            color = colors.get(camera_id, (100, 100, 100))
            
            # Create frame with OpenCV
            import numpy as np
            frame = np.full((height, width, 3), color, dtype=np.uint8)
            
            # Add text
            font = cv2.FONT_HERSHEY_SIMPLEX
            text = f"Demo Camera {camera_id}"
            text_size = cv2.getTextSize(text, font, 1, 2)[0]
            text_x = (width - text_size[0]) // 2
            text_y = (height + text_size[1]) // 2
            cv2.putText(frame, text, (text_x, text_y), font, 1, (255, 255, 255), 2)
            
            # Add frame counter
            counter_text = f"Frame: {self.frame_count}"
            cv2.putText(frame, counter_text, (10, 30), font, 0.7, (255, 255, 255), 2)
            
            # Add timestamp
            timestamp = time.strftime("%H:%M:%S")
            cv2.putText(frame, timestamp, (10, height - 10), font, 0.7, (255, 255, 255), 2)
            
            # Add a moving circle
            import math
            circle_x = int(50 + 200 * (0.5 + 0.5 * math.sin(self.frame_count * 0.1)))
            circle_y = int(height // 2 + 50 * math.sin(self.frame_count * 0.2))
            cv2.circle(frame, (circle_x, circle_y), 20, (255, 255, 255), -1)
            
            # Encode to JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            if ret:
                self.last_frame_time = time.time()
                return buffer.tobytes()
            return None
            
        def get_frame_base64(self):
            frame_bytes = self.get_frame()
            if frame_bytes:
                return base64.b64encode(frame_bytes).decode('utf-8')
            return None
            
        def is_alive(self):
            return self.running
            
        def stop(self):
            self.running = False
    
    try:
        camera = DemoCamera(camera_id)
        active_cameras[camera_id] = camera
        print(f"Created demo camera {camera_id}")
        return True
    except Exception as e:
        print(f"Failed to create demo camera {camera_id}: {e}")
        return False

def start_camera(camera_id):
    """Start a camera stream"""
    if camera_id in active_cameras:
        return True
    
    rtsp_url = f"rtsp://127.0.0.1:8554/cam_{camera_id}"
    camera = CameraStream(camera_id, rtsp_url)
    
    if camera.start():
        active_cameras[camera_id] = camera
        print(f"Started camera {camera_id} at {rtsp_url}")
        return True
    else:
        print(f"Failed to start camera {camera_id}")
        # For demo purposes, create a fake camera stream
        print(f"Creating demo stream for camera {camera_id}")
        return create_demo_camera(camera_id)

def stop_camera(camera_id):
    """Stop a camera stream"""
    if camera_id in active_cameras:
        active_cameras[camera_id].stop()
        del active_cameras[camera_id]
        print(f"Stopped camera {camera_id}")

def generate_camera_feed(camera_id):
    """Generate camera feed for streaming"""
    camera = active_cameras.get(camera_id)
    if not camera:
        # Try to start the camera if it's not running
        if not start_camera(camera_id):
            return
        camera = active_cameras.get(camera_id)
    
    while camera and camera.is_alive():
        frame_bytes = camera.get_frame()
        if frame_bytes:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.1)  # 10 FPS

# Mock database for robot data (in production, replace with actual database)
robot_data = {
    "robots": [
        {
            "id": "alpha",
            "name": "Robot Alpha",
            "group": "production",
            "status": "active",
            "emoji": "🟢",
            "path": "Gallery Tour A",
            "battery": 85,
            "uptime": "2h 15m",
            "position": {"x": 0.4, "y": 0.5, "angle": 45},
            "last_seen": datetime.now().isoformat()
        },
        {
            "id": "beta",
            "name": "Robot Beta",
            "group": "production",
            "status": "idle",
            "emoji": "🌙",
            "path": "Idle",
            "battery": 92,
            "uptime": "45m",
            "position": {"x": 0.6, "y": 0.4, "angle": 180},
            "last_seen": datetime.now().isoformat()
        },
        {
            "id": "gamma",
            "name": "Robot Gamma",
            "group": "production",
            "status": "error",
            "emoji": "⚠️",
            "path": "Gallery Tour B",
            "battery": 67,
            "issue": "Navigation Error",
            "position": {"x": 0.35, "y": 0.65, "angle": 90},
            "last_seen": datetime.now().isoformat()
        },
        {
            "id": "delta",
            "name": "Robot Delta",
            "group": "testing",
            "status": "offline",
            "emoji": "🛑",
            "path": "--",
            "battery": "--",
            "issue": "Offline",
            "position": {"x": 0.2, "y": 0.3, "angle": 0},
            "last_seen": (datetime.now() - timedelta(hours=2)).isoformat()
        },
        {
            "id": "echo",
            "name": "Robot Echo",
            "group": "testing",
            "status": "warning",
            "emoji": "❗",
            "path": "Test Route B",
            "battery": 78,
            "warning": "Low Speed",
            "position": {"x": 0.8, "y": 0.3, "angle": 270},
            "last_seen": datetime.now().isoformat()
        },
        {
            "id": "zeta",
            "name": "Robot Zeta",
            "group": "ungrouped",
            "status": "standby",
            "emoji": "🟡",
            "path": "Standby",
            "battery": 100,
            "status_text": "Ready",
            "position": {"x": 0.7, "y": 0.7, "angle": 0},
            "last_seen": datetime.now().isoformat()
        },
        {
            "id": "theta",
            "name": "Robot Theta",
            "group": "ungrouped",
            "status": "sleeping",
            "emoji": "💤",
            "path": "Sleep Mode",
            "battery": 95,
            "status_text": "Charging",
            "position": {"x": 0.2, "y": 0.8, "angle": 135},
            "last_seen": datetime.now().isoformat()
        }
    ],
    "groups": [
        {
            "id": "production",
            "name": "Production Robots",
            "expanded": True,
            "robot_count": 3
        },
        {
            "id": "testing",
            "name": "Testing Robots",
            "expanded": False,
            "robot_count": 2
        }
    ],
    "cameras": [
        {
            "id": "1",
            "name": "Camera 1",
            "source": "cam_1",
            "available_sources": ["cam_1", "cam_2", "cam_3", "cam_4"],
            "stream_url": "/api/camera/1/stream",
            "rtsp_url": "rtsp://127.0.0.1:8554/cam_1",
            "status": "offline"
        },
        {
            "id": "2",
            "name": "Camera 2", 
            "source": "cam_2",
            "available_sources": ["cam_1", "cam_2", "cam_3", "cam_4"],
            "stream_url": "/api/camera/2/stream",
            "rtsp_url": "rtsp://127.0.0.1:8554/cam_2",
            "status": "offline"
        },
        {
            "id": "3",
            "name": "Camera 3",
            "source": "cam_3", 
            "available_sources": ["cam_1", "cam_2", "cam_3", "cam_4"],
            "stream_url": "/api/camera/3/stream",
            "rtsp_url": "rtsp://127.0.0.1:8554/cam_3",
            "status": "offline"
        },
        {
            "id": "4",
            "name": "Camera 4",
            "source": "cam_4", 
            "available_sources": ["cam_1", "cam_2", "cam_3", "cam_4"],
            "stream_url": "/api/camera/3/stream",
            "rtsp_url": "rtsp://127.0.0.1:8554/cam_4",
            "status": "offline"
        }
    ],
    "map": {
        "pillars": [
            {"x": 0.25, "y": 0.25},
            {"x": 0.75, "y": 0.25},
            {"x": 0.25, "y": 0.75},
            {"x": 0.75, "y": 0.75},
            {"x": 0.5, "y": 0.5}
        ],
        "paths": {
            "alpha": [
                {"x": 0.4, "y": 0.5},
                {"x": 0.5, "y": 0.4},
                {"x": 0.6, "y": 0.5},
                {"x": 0.7, "y": 0.6},
                {"x": 0.5, "y": 0.7},
                {"x": 0.3, "y": 0.6}
            ],
            "gamma": [
                {"x": 0.35, "y": 0.65},
                {"x": 0.4, "y": 0.6},
                {"x": 0.45, "y": 0.55}
            ],
            "echo": [
                {"x": 0.8, "y": 0.3},
                {"x": 0.7, "y": 0.4},
                {"x": 0.6, "y": 0.6},
                {"x": 0.8, "y": 0.7}
            ]
        }
    }
}

# API Routes

@api_bp.route('/robots', methods=['GET'])
def get_robots():
    """Get all robot data"""
    return jsonify(robot_data)

@api_bp.route('/robots/<robot_id>', methods=['GET'])
def get_robot(robot_id):
    """Get specific robot data"""
    robot = next((r for r in robot_data["robots"] if r["id"] == robot_id), None)
    if not robot:
        return jsonify({"error": "Robot not found"}), 404
    return jsonify(robot)

@api_bp.route('/robots/<robot_id>', methods=['PUT'])
def update_robot(robot_id):
    """Update robot data"""
    robot = next((r for r in robot_data["robots"] if r["id"] == robot_id), None)
    if not robot:
        return jsonify({"error": "Robot not found"}), 404
    
    data = request.get_json()
    if not data:
        return jsonify({"error": "No data provided"}), 400
    
    # Update allowed fields
    allowed_fields = ['name', 'group', 'status', 'path', 'battery', 'position']
    for field in allowed_fields:
        if field in data:
            robot[field] = data[field]
    
    robot['last_seen'] = datetime.now().isoformat()
    return jsonify(robot)

@api_bp.route('/robots/<robot_id>/move', methods=['POST'])
def move_robot(robot_id):
    """Move robot to different group"""
    robot = next((r for r in robot_data["robots"] if r["id"] == robot_id), None)
    if not robot:
        return jsonify({"error": "Robot not found"}), 404
    
    data = request.get_json()
    new_group = data.get('group')
    if not new_group:
        return jsonify({"error": "Group is required"}), 400
    
    robot['group'] = new_group
    robot['last_seen'] = datetime.now().isoformat()
    return jsonify({"message": "Robot moved successfully", "robot": robot})

@api_bp.route('/groups', methods=['GET'])
def get_groups():
    """Get all robot groups"""
    return jsonify(robot_data["groups"])

@api_bp.route('/groups/<group_id>', methods=['PUT'])
def update_group(group_id):
    """Update group settings"""
    group = next((g for g in robot_data["groups"] if g["id"] == group_id), None)
    if not group:
        return jsonify({"error": "Group not found"}), 404
    
    data = request.get_json()
    if not data:
        return jsonify({"error": "No data provided"}), 400
    
    # Update allowed fields
    allowed_fields = ['name', 'expanded']
    for field in allowed_fields:
        if field in data:
            group[field] = data[field]
    
    return jsonify(group)

@api_bp.route('/cameras', methods=['GET'])
def get_cameras():
    """Get all camera data with live status"""
    cameras = robot_data["cameras"].copy()
    for camera in cameras:
        camera_id = camera["id"]
        if camera_id in active_cameras and active_cameras[camera_id].is_alive():
            camera["status"] = "online"
        else:
            camera["status"] = "offline"
    return jsonify(cameras)

@api_bp.route('/cameras/<camera_id>', methods=['PUT'])
def update_camera(camera_id):
    """Update camera settings"""
    camera = next((c for c in robot_data["cameras"] if c["id"] == camera_id), None)
    if not camera:
        return jsonify({"error": "Camera not found"}), 404
    
    data = request.get_json()
    if not data:
        return jsonify({"error": "No data provided"}), 400
    
    # Update allowed fields
    allowed_fields = ['name', 'source']
    for field in allowed_fields:
        if field in data:
            camera[field] = data[field]
    
    return jsonify(camera)

@api_bp.route('/camera/<camera_id>/stream')
def camera_stream(camera_id):
    """Live camera stream endpoint"""
    try:
        return Response(generate_camera_feed(camera_id),
                       mimetype='multipart/x-mixed-replace; boundary=frame')
    except Exception as e:
        print(f"Error streaming camera {camera_id}: {e}")
        return jsonify({"error": "Camera stream unavailable"}), 503

@api_bp.route('/camera/<camera_id>/start', methods=['POST'])
def start_camera_endpoint(camera_id):
    """Start a camera stream"""
    if start_camera(camera_id):
        return jsonify({"message": f"Camera {camera_id} started successfully"})
    else:
        return jsonify({"error": f"Failed to start camera {camera_id}"}), 500

@api_bp.route('/camera/<camera_id>/stop', methods=['POST'])
def stop_camera_endpoint(camera_id):
    """Stop a camera stream"""
    stop_camera(camera_id)
    return jsonify({"message": f"Camera {camera_id} stopped"})

@api_bp.route('/camera/<camera_id>/frame')
def get_camera_frame(camera_id):
    """Get a single frame from camera as base64"""
    camera = active_cameras.get(camera_id)
    if not camera:
        if not start_camera(camera_id):
            return jsonify({"error": "Camera not available"}), 404
        camera = active_cameras.get(camera_id)
    
    if camera and camera.is_alive():
        frame_base64 = camera.get_frame_base64()
        if frame_base64:
            return jsonify({
                "camera_id": camera_id,
                "frame": frame_base64,
                "timestamp": time.time()
            })
    
    return jsonify({"error": "No frame available"}), 404

@api_bp.route('/camera/<camera_id>/status')
def get_camera_status(camera_id):
    """Get camera status"""
    camera = next((c for c in robot_data["cameras"] if c["id"] == camera_id), None)
    if not camera:
        return jsonify({"error": "Camera not found"}), 404
    
    is_active = camera_id in active_cameras and active_cameras[camera_id].is_alive()
    
    return jsonify({
        "camera_id": camera_id,
        "name": camera["name"],
        "status": "online" if is_active else "offline",
        "rtsp_url": camera["rtsp_url"],
        "last_frame": active_cameras[camera_id].last_frame_time if is_active else None
    })

@api_bp.route('/cameras/init', methods=['POST'])
def init_cameras():
    """Initialize all available cameras"""
    results = {}
    for camera_data in robot_data["cameras"]:
        camera_id = camera_data["id"]
        success = start_camera(camera_id)
        results[camera_id] = {
            "success": success,
            "rtsp_url": camera_data["rtsp_url"],
            "status": "started" if success else "failed"
        }
    
    return jsonify({
        "message": "Camera initialization complete",
        "results": results
    })

@api_bp.route('/cameras/test', methods=['GET'])
def test_cameras():
    """Test connectivity to all camera streams"""
    results = {}
    
    for camera_data in robot_data["cameras"]:
        camera_id = camera_data["id"]
        rtsp_url = camera_data["rtsp_url"]
        
        try:
            # Try to open the camera stream briefly
            cap = cv2.VideoCapture(rtsp_url)
            if cap.isOpened():
                ret, frame = cap.read()
                success = ret and frame is not None
                cap.release()
            else:
                success = False
            
            results[camera_id] = {
                "rtsp_url": rtsp_url,
                "accessible": success,
                "status": "accessible" if success else "not_accessible"
            }
        except Exception as e:
            results[camera_id] = {
                "rtsp_url": rtsp_url,
                "accessible": False,
                "status": "error",
                "error": str(e)
            }
    
    return jsonify({
        "message": "Camera connectivity test complete",
        "results": results
    })

@api_bp.route('/map', methods=['GET'])
def get_map():
    """Get map data including pillars and robot paths"""
    return jsonify(robot_data["map"])

@api_bp.route('/status', methods=['GET'])
def get_system_status():
    """Get overall system status"""
    total_robots = len(robot_data["robots"])
    active_robots = len([r for r in robot_data["robots"] if r["status"] == "active"])
    offline_robots = len([r for r in robot_data["robots"] if r["status"] == "offline"])
    error_robots = len([r for r in robot_data["robots"] if r["status"] in ["error", "warning"]])
    
    return jsonify({
        "total_robots": total_robots,
        "active_robots": active_robots,
        "offline_robots": offline_robots,
        "error_robots": error_robots,
        "system_status": "operational" if error_robots == 0 else "degraded",
        "last_update": datetime.now().isoformat()
    })

# Real-time updates endpoint
@api_bp.route('/updates', methods=['GET'])
def get_updates():
    """Get recent updates/changes"""
    # In a real system, this would return actual updates from the last poll
    return jsonify({
        "updates": [],
        "timestamp": datetime.now().isoformat()
    })

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
        return {"message": "Notification sent"}, 200
    print(r.text)
    return {"message": "Failed to send notification"}, 500