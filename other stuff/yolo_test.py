import cv2

from ultralytics import YOLO

# Load a pretrained YOLO11n model
model = YOLO("yolo11n.pt")

# Read an image using OpenCV
cap = cv2.VideoCapture("rtsp://127.0.0.1:8554/cam_4")

# Run inference on the source
while True:
    ret, frame = cap.read()
    frame = cv2.resize(frame, (640, 480))
    if not ret:
        continue
    results = model(frame)  # list of Results objects
    annotated_frame = results[0].plot()  # annotated image
    cv2.imshow("YOLO11n Inference", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()