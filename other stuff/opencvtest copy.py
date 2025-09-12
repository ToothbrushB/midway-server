from time import sleep
import cv2
import numpy as np
import json


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

cap = cv2.VideoCapture("rtsp://127.0.0.1:8554/cam_1")
if not cap.isOpened():
    print("Error: Could not open video capture.")
    exit()

# First calculate homography matrix
homoFrame = None
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break
    cv2.imshow("Click q when ready to capture", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        homoFrame = frame
        cv2.destroyWindow("Click q when ready to capture")
        break
if homoFrame is None:
    print("Error: No frame captured.")
    exit()

# homoFrame = cv2.undistort(homoFrame, camera_matrix, dist_coeffs)
# prompt user to select 4 points on the frame
cv2.imshow("Select 4 points", homoFrame)

# points = []
# # Define a mouse callback function to capture points
# def select_point(event, x, y, flags, param):
#     global points
#     if event == cv2.EVENT_LBUTTONDOWN:
#         cv2.circle(homoFrame, (x, y), 5, (0, 255, 0), -1)
#         # draw a line between adjacent points
#         if len(points) > 0:
#             cv2.line(homoFrame, points[-1], (x, y), (0, 255, 0), 2)
#         cv2.imshow("Select 4 points", homoFrame)
#         points.append((x, y))
#         if len(points) == 4:
#             cv2.destroyWindow("Select 4 points")
# cv2.setMouseCallback("Select 4 points", select_point)

# while len(points) < 4:
#     cv2.waitKey(1)

points = []
drawing_done = False

def select_point(event, x, y, flags, param):
    global points, drawing_done
    if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
        points.append((x, y))
        if len(points) == 4:
            drawing_done = True

cv2.setMouseCallback("Select 4 points", select_point)

while not drawing_done:
    display = homoFrame.copy()
    for idx, pt in enumerate(points):
        cv2.circle(display, pt, 5, (0, 255, 0), -1)
        if idx > 0:
            cv2.line(display, points[idx-1], pt, (0, 255, 0), 2)
    cv2.imshow("Select 4 points", display)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyWindow("Select 4 points")
print(points)


# H = cv2.findHomography(np.array(points, dtype=np.float32), 
#                    np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32), method=cv2.RANSAC)
# # cv2.solvePnP
# if H is None:
#     print("Error: Could not compute homography matrix.")
#     exit()
# print(H)

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print("Error: Could not read frame.")
#         break
#     # Undistort the frame using the camera matrix and distortion coefficients
#     frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

#     for color, (lower, upper) in COLOR_RANGES.items():
#         lower = np.array(lower, dtype=np.uint8)
#         upper = np.array(upper, dtype=np.uint8)
#         mask = cv2.inRange(hsv, lower, upper)
#     # https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html#ga819779b9857cc2f8601e6526a3a5bc71 and https://docs.opencv.org/4.x/d4/d73/tutorial_py_contours_begin.html
#         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         contours = [c for c in contours if cv2.contourArea(c) > 100]
#         cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)

#     # Use the homography matrix to project a coordinate axes at 0, 0
#     axes_points = np.array([
#         [0, 0],    # origin
#         [1, 0],  # x-axis
#         [0, 1]   # y-axis
#     ], dtype=np.float32).reshape(-1, 1, 2)
#     img_points = cv2.perspectiveTransform(axes_points, np.array(H[0]))  # H is a tuple (matrix, mask)
#     img_points = img_points.astype(int)

#     origin = tuple(img_points[0][0])
#     x_axis = tuple(img_points[1][0])
#     y_axis = tuple(img_points[2][0])

#     print(f"Origin: {origin}, X-axis: {x_axis}, Y-axis: {y_axis}")
#     # Draw axes
#     cv2.arrowedLine(frame, origin, x_axis, (0, 0, 255), 3)  # X axis in red
#     cv2.arrowedLine(frame, origin, y_axis, (0, 255, 0), 3)  # Y axis in green


#     cv2.imshow("Undistorted Frame", frame)
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break
# cap.release()
# cv2.destroyAllWindows()