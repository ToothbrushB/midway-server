from ultralytics import YOLO
import cv2
import numpy as np
from time import sleep
edgetpu_model = YOLO("0911edgetpu.tflite")

cap = cv2.VideoCapture("rtsp://127.0.0.1:8554/cam_4")
ret, frame = cap.read()
timer = 0
while timer < 150:
    ret, frame = cap.read()
    timer += 1

ret, frame = cap.read()
cv2.imwrite("thepicture4.png", frame)

# cap.release()

# results = edgetpu_model(frame)
# print(results)
# exit()

# H = np.load("H_cam3.npy")
# camera_matrix = np.load('./camera_matrix.npy')
# dist_coeffs = np.load('./dist_coeffs.npy')

# undistorted_3 = cv2.undistortPoints(cam3_pts, camera_matrix, dist_coeffs, P=camera_matrix)
# ones = np.ones((undistorted_3.shape[0], 1), dtype=np.float32)
# undistorted_3_hom = np.hstack([undistorted_3, ones])  # Shape (N, 3)

# # Apply homography
# global_pts_hom = (H @ undistorted_3_hom.T).T  # Shape (N, 3)