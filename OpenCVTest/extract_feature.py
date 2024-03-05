import cv2
from math import floor, hypot
from random import randint
from time import time
from os import mkdir
from datetime import datetime
# import numpy as np
# import sys

## Import face, setup outputs
current_timestamp = int(time())

inputImagePath = "assets/three_faces.jpg"
outputPath = f"outputs/output_{current_timestamp}"

mkdir(outputPath)

image = cv2.imread(inputImagePath)
cv2.imwrite(outputPath + '/input.jpg', image)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
# cv2.imwrite(outputPath + '/input_grey.jpg', gray)

## Setup detector
t1 = datetime.now()
faceCascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
faces = faceCascade.detectMultiScale(
  gray,
  scaleFactor=1.3,
  minNeighbors=3,
  minSize=(30, 30)
)
t2 = datetime.now()

print(f"[INFO] Found {len(faces)} faces in {(t2-t1)}")

## Find biggest face
perc_increase : float = 0.3
biggest_face = [0, None]
img_found_faces = image.copy()
for face in faces:
  (x, y, w, h) = face
  cv2.rectangle(img_found_faces, (x,y), (x+w,y+h), (0, 255, 0), 2)
  size = hypot(w,h)
  if size > biggest_face[0]:
    biggest_face = [size, face]
    w_increase : int = floor(w * perc_increase)
    h_increase : int = floor(h * perc_increase)

cv2.imwrite(outputPath + '/detected_faces.jpg', img_found_faces)

## Crop to biggest face
(x, y, w, h) = biggest_face[1]
w_increase : int = floor(w * perc_increase)
h_increase : int = floor(h * perc_increase)
crop_img = image[y-h_increase:y+h+h_increase, x-w_increase:x+w+w_increase]

cv2.imwrite(outputPath + '/output.jpg', crop_img)
