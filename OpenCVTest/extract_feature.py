import cv2
from math import floor, hypot, atan2, degrees
# from random import randint
from time import time
from os import mkdir
from datetime import datetime
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev

images = []

## Import face, setup outputs
current_timestamp = int(time())

# inputImagePath = "assets/three_faces.jpg"
# inputImagePath = "assets/easy.jpg"
inputImagePath = "assets/easy-vectorised.jpg"
outputPath = f"outputs/output_{current_timestamp}"

mkdir(outputPath)

img = cv2.imread(inputImagePath)
images.append(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
# cv2.imwrite(outputPath + '/input.jpg', img)
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# images.append(img_gray)

## Setup detector
t1 = datetime.now()
faceCascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
faces = faceCascade.detectMultiScale(
  img_gray,
  scaleFactor=1.3,
  minNeighbors=3,
  minSize=(30, 30)
)
t2 = datetime.now()

print(f"[INFO] Found {len(faces)} faces in {(t2-t1)}")

## Find biggest face
perc_increase : float = 0.35
biggest_face = [0, None]
img_found_faces = img_gray.copy()
for face in faces:
  (x, y, w, h) = face
  cv2.rectangle(img_found_faces, (x,y), (x+w,y+h), (0, 255, 0), 2)
  size = hypot(w,h)
  if size > biggest_face[0]:
    biggest_face = [size, face]
    w_increase : int = floor(w * perc_increase)
    h_increase : int = floor(h * perc_increase)

# cv2.imwrite(outputPath + '/detected_faces.jpg', img_found_faces)
images.append(img_found_faces)

## Crop to biggest face
(x, y, w, h) = biggest_face[1]
w_increase : int = floor(w * perc_increase)
h_increase : int = floor(h * perc_increase)
crop_img = img_gray[y-h_increase:y+h+h_increase, x-w_increase:x+w+w_increase]
crop_img_rgb = img.copy()[y-h_increase:y+h+h_increase, x-w_increase:x+w+w_increase]
images.append(crop_img_rgb)

# img_hsv = cv2.cvtColor(crop_img_rgb, cv2.COLOR_BGR2HSV)
# images.append(img_hsv)

# images.append(foreground)

# blurred = cv2.GaussianBlur(crop_img_rgb, (3 , 3), 0)
# # images.append(blurred)
# edges = cv2.Canny(crop_img, 80, 130)
# # cv2.imwrite(outputPath + '/edges.jpg', edges)


# # blurred2 = cv2.GaussianBlur(edges, (3 , 3), 0)
# # images.append(blurred2)
# contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# contours = [contour for contour in contours if cv2.contourArea(contour, True)<0]

# cv2.drawContours(crop_img_rgb, contours, -1, (255, 0, 0), 1) 
# images.append(cv2.cvtColor(crop_img_rgb, cv2.COLOR_BGR2RGB))
# images.append(cv2.bitwise_not(edges))

fig = plt.figure(figsize=(16, 8))
fign = 1
columns = 4
rows = int(len(images) / columns)+1
for i,img in enumerate(images):
    fig.add_subplot(rows, columns, fign)
    fign += 1
    plt.imshow(img, cmap = plt.cm.gray)


# def calculate_angle(p1, p2):
#     """Calculate the angle between two points from the horizontal."""
#     dx = p2[0] - p1[0]
#     dy = p2[1] - p1[1]
#     return degrees(atan2(dy, dx))

# def split_on_direction_change(contour, max_dir_change):
#     """Split contour array if direction change exceeds max_dir_change degrees."""
#     splits = []
#     current = [contour[0]]  # Initialize with the first point

#     for i in range(1, len(contour) - 1):
#         angle1 = calculate_angle(contour[i-1][0], contour[i][0])
#         angle2 = calculate_angle(contour[i][0], contour[i+1][0])
#         angle_change = abs(angle2 - angle1)
        
#         # Normalize the angle change to be within 180 degrees
#         angle_change = min(angle_change, 360 - angle_change)
        
#         if angle_change > max_dir_change:
#             # Add the current point to both parts of the split
#             current.append(contour[i])
#             splits.append(current)
#             current = [contour[i]]
#         else:
#             current.append(contour[i])
    
#     # Add the last segment
#     current.append(contour[-1])
#     splits.append(current)
    
#     return splits

# n = 1  # Select every nth element
# m = 20  # Max elements in a split array
# min_contour_len = 7  
# max_dir_change = 90  # Maximum allowed direction change in degrees

# new_contours = []

# # Process original contours
# for contour in contours:
#     selected = contour[::n]
#     for i in range(0, len(selected), m):
#         split_array = selected[i:i + m]
#         new_contours.append(split_array)

# processed_contours = []

# # Split contours based on direction change
# for contour in new_contours:
#     split_contours = split_on_direction_change(contour, max_dir_change)
#     processed_contours.extend(split_contours)
    
# fig.add_subplot(rows, columns, fign)
# fign += 1

# for contour in processed_contours:
#     points = np.array(contour).reshape(-1, 2)
#     plt.plot(points[:, 0], -points[:, 1])

# def smooth_contour(contour):
#     """
#     Smooth a given contour by approximating it with a 3rd degree polynomial curve in 3D,
#     using the index as the x-value, and then project it back to a 2D curve.
    
#     Parameters:
#     - contour: The input contour, a list of [x, y] points.
    
#     Returns:
#     - smoothed_contour: The smoothed 2D contour as a list of [x, y] points.
#     """
#     # Convert contour to an appropriate format and include index as x value
#     x = np.arange(len(contour))
#     y = np.array(contour).reshape(-1, 2).T  # Transpose to have 2 rows: one for x and one for y
    
#     # Fit a spline to the 3D curve (index, x, y)
#     tck, u = splprep([x, y[0], y[1]], s=3, k=3)
    
#     # Evaluate the spline across the original parameter range
#     new_points = splev(u, tck)
    
#     # Extract the smoothed curve, ignoring the first dimension (index)
#     smoothed_contour = np.vstack(new_points[1:]).T  # Transpose back to original shape
    
#     return smoothed_contour

# processed_contours = [smooth_contour(contour) for contour in processed_contours if len(contour) >= min_contour_len]

# # Plotting
# fig.add_subplot(rows, columns, fign)
# fign += 1

# for contour in processed_contours:
#     points = np.array(contour).reshape(-1, 2)
#     plt.plot(points[:, 0], -points[:, 1])
    
# import os
    
# output_file_path = os.path.join(outputPath, "output.txt")

# # Exporting the array to the file
# with open(output_file_path, "w") as file:
#     for contour in processed_contours:
#         file.write(str(contour) + '\n')

plt.axis('equal')
plt.show()
