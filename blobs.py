import cv2
import numpy as np
import random

# --- Load your camera feed or image ---
# cap = cv2.VideoCapture(0)  # change index if using another camera

# --- Set up blob detector parameters ---
# params = cv2.SimpleBlobDetector_Params()
# params.filterByColor = True
# params.blobColor = 255  # bright blobs (e.g. white/LED)

# params.filterByArea = True
# params.minArea = 20
# params.maxArea = 5000

# params.filterByCircularity = False
# params.filterByConvexity = False
# params.filterByInertia = False

# # Create detector
# detector = cv2.SimpleBlobDetector_create(params)

img = cv2.imread("blobs.jpg")
# img = cv2.resize(img, (1280, 960))  # Resize for better visibility
# Convert to grayscale and threshold
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Histogram equalization to enhance contrast
gray_eq = cv2.equalizeHist(gray)

scale = 3.125
# Gaussian blur to suppress noise
blurred = cv2.GaussianBlur(gray_eq, (13, 13), 2*scale)

# Edge detection
# edges = cv2.Canny(blurred, 30, 100)

# Detect circles
circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.5, minDist=40*scale,
                           param1=120, param2=50,
                           minRadius=round(20*scale), maxRadius=round(100*scale))
# print(circles)
# filter out circles that are concentric
# if circles is not None:
#     circles = np.uint16(np.around(circles))
#     filtered_circles = []
#     for i in range(len(circles[0])):
#         x, y, r = circles[0][i]
#         if not any(np.linalg.norm(np.array([x, y]) - np.array([c[0], c[1]])) < 10 for c in filtered_circles):
#             filtered_circles.append((x, y, r))
#     circles = np.array(filtered_circles).reshape(-1, 3)
# print(circles)
# Draw detected circles
output = img.copy()
if circles is not None:
    for (x, y, r) in circles[0, :]:
        cv2.circle(output, (int(x), int(y)), int(r), (0, 255, 0), 2)
        cv2.circle(output, (int(x), int(y)), 2, (0, 0, 255), 3)


# _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

# # Detect blobs
# keypoints = detector.detect(thresh)

# # Draw detected blobs
# annotated = cv2.drawKeypoints(
#     frame, keypoints, np.array([]),
#     (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
# )

# # Print positions
# for i, k in enumerate(keypoints):
#     x, y = k.pt
#     print(f"Blob {i}: x={x:.1f}, y={y:.1f}, size={k.size:.1f}")

blurredsmall = cv2.resize(blurred, (1280, 960))
outputsmall = cv2.resize(output, (1280, 960))
cv2.imshow("Blobs", blurredsmall)                                                                                               
while not cv2.waitKey(1) & 0xFF == ord('q'):
    pass
cv2.imshow("Blobs", outputsmall)
while not cv2.waitKey(1) & 0xFF == ord('q'):
    pass
cv2.destroyAllWindows()
