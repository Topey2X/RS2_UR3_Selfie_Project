from img_processing_functions import *

import cv2
import numpy as np
import matplotlib.pyplot as plt


def test_face_detection():
  print("TEST1: FACIAL RECOGNITION TEST")
  minimum_accuracy = 90.0
  # try:
  images = [
    ("tests/test1/img1_test.jpg", "tests/test1/img1_truth.png"),
    ("tests/test1/img2_test.jpg", "tests/test1/img2_truth.png"),
    ("tests/test1/img3_test.jpg", "tests/test1/img3_truth.png"),
    ("tests/test1/img4_test.jpg", "tests/test1/img4_truth.png"),
    ("tests/test1/img5_test.jpg", "tests/test1/img5_truth.png"),    
  ]
  accuracy = []
  images_output = []
  for i,(test_img_name, truth_img_name) in enumerate(images):
    try:
      test_img = cv2.imread(test_img_name)
      rect = identify_faces(convert_img_to_grayscale(test_img))  # Expected to return (x, y, w, h)
      
      truth_img = cv2.imread(
        truth_img_name
      ) # BGR
      
      (x, y, w, h) = rect
      cv2.rectangle(test_img, (x,y), (x+w,y+h), (0, 255, 0), 2)
      images_output.append(test_img)

      # Initialize counters
      TP, FP, TN, FN = 0, 0, 0, 0
      w = truth_img.shape[1]
      h = truth_img.shape[0]
      for x in range(w):  # Image width
        for y in range(h):  # Image height
          # Determine if the current pixel is inside the identified rectangle
          inside = lambda: (
            rect[0] <= x <= (rect[0] + rect[2])
            and rect[1] <= y <= (rect[1] + rect[3])
          )
          color = truth_img[y, x]

          if color[1] == 255 and color[2] == 0 and color[0] == 0:  # Green pixel
            if inside():
              TP += 1
            else:
              FN += 1
          elif color[2] == 255 and color[1] == 0 and color[0] == 0:  # Red pixel
            if inside():
              FP += 1
            else:
              TN += 1

      a = ((TP+TN) / (TP+TN+FN+FP)) * 100
      accuracy.append(a)
      # Print results
      print(f"\tSUBTEST {i+1}: {a}% ({TP}-{FP}/{TN}-{FN})")
    except Exception as e:
      # print(f"\tSUBTEST FAILED: Error ({repr(e)})")
      accuracy.append(0.0)
      print(f"\tSUBTEST FAILED: Error")
      
  
  
  worst_accuracy = min(accuracy)
  if worst_accuracy >= minimum_accuracy:
    print(f"TEST PASSED (worst case {worst_accuracy}%)")
  else:
    print(f"TEST FAILED (worst case {worst_accuracy}%)")
    
  fig = plt.figure(figsize=(16, 8))
  fign = 1
  columns = 5
  rows = 1
  for i,img in enumerate(images_output):
    fig.add_subplot(rows, columns, fign)
    fign += 1
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
  plt.show()
    


def tests():
    # Test 1: Face Detect
    test_face_detection()


if __name__ == "__main__":
    tests()
