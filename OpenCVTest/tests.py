from lib_img_processing import *

import cv2
import numpy as np
import matplotlib.pyplot as plt


def test_face_detection() -> bool:
  print("\nTEST 1: FACIAL RECOGNITION TEST")
  minimum_accuracy = 90.0
  # try:
  images = [
    ("OpenCVTest/tests/test1/img1_test.jpg", "OpenCVTest/tests/test1/img1_truth.png"),
    ("OpenCVTest/tests/test1/img2_test.jpg", "OpenCVTest/tests/test1/img2_truth.png"),
    ("OpenCVTest/tests/test1/img3_test.jpg", "OpenCVTest/tests/test1/img3_truth.png"),
    ("OpenCVTest/tests/test1/img4_test.jpg", "OpenCVTest/tests/test1/img4_truth.png"),
    ("OpenCVTest/tests/test1/img5_test.jpg", "OpenCVTest/tests/test1/img5_truth.png"),    
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
  result = worst_accuracy >= minimum_accuracy
  if result:
    print(f"TEST PASSED (worst case {worst_accuracy}%)")
  else:
    print(f"TEST FAILED (worst case {worst_accuracy}%)")
    
  fig = plt.figure(figsize=(16, 8))
  fig.suptitle("Face Detection Test")
  fign = 1
  columns = 5
  rows = 1
  for i,img in enumerate(images_output):
    fig.add_subplot(rows, columns, fign)
    fign += 1
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
  
  return result
  
def test_edge_detection() -> bool:
    print("\nTEST 2: EDGE DETECTION TEST")
    minimum_accuracy = 90.0
    images = [
        ("OpenCVTest/tests/test2/img1_test.png", "OpenCVTest/tests/test2/img1_truth.png"),
        ("OpenCVTest/tests/test2/img2_test.png", "OpenCVTest/tests/test2/img2_truth.png"),
        ("OpenCVTest/tests/test2/img3_test.png", "OpenCVTest/tests/test2/img3_truth.png"), 
    ]
    accuracy = []
    images_output = []

    for i, (test_img_name, truth_img_name) in enumerate(images):
        try:
            # Load images
            test_img = cv2.imread(test_img_name)
            truth_img = cv2.imread(truth_img_name)

            # Process the test image to detect edges
            binary_img = extract_edges_from_face(test_img, 80, 130)

            # Prepare the mask for green pixels in truth image
            green_mask = (truth_img[:, :, 1] == 255) & (truth_img[:, :, 0] == 0) & (truth_img[:, :, 2] == 0)

            # Calculate accuracy
            white_pixels = (binary_img == 255)
            correct_pixels = np.logical_and(white_pixels, green_mask)
            a = (np.sum(correct_pixels) / np.sum(white_pixels)) * 100
            accuracy.append(a)

            # Output the subtest result
            print(f"\tSUBTEST {i+1}: {a}%")
            images_output.append((test_img, binary_img, truth_img))
        except Exception as e:
            accuracy.append(0.0)
            print(f"\tSUBTEST FAILED: Error")

    # Determine the overall test result
    worst_accuracy = min(accuracy)
    result = worst_accuracy >= minimum_accuracy
    if result:
        print(f"TEST PASSED (worst case {worst_accuracy}%)")
    else:
        print(f"TEST FAILED (worst case {worst_accuracy}%)")
    
    # Display the images
    fig = plt.figure(figsize=(16, 8))
    fig.suptitle("Edge Detection Test")
    fign = 1
    columns = 3
    rows = len(images)
    for test_img, binary_img, truth_img in images_output:
        fig.add_subplot(rows, columns, fign)
        plt.imshow(cv2.cvtColor(test_img, cv2.COLOR_BGR2RGB))
        fign += 1
        fig.add_subplot(rows, columns, fign)
        plt.imshow(binary_img, cmap='gray')
        fign += 1
        fig.add_subplot(rows, columns, fign)
        plt.imshow(cv2.cvtColor(truth_img, cv2.COLOR_BGR2RGB))
        fign += 1

    return result

def test_contour_detect() -> bool:
  return True

def test_optimisation() -> bool:
  return True

def test_background_removal() -> bool:
  print("\nTEST 5: BACKGROUND REMOVAL TEST")
  minimum_accuracy = 80.0
  threshold = 0xC8
  images = [
    ("OpenCVTest/tests/test5/img1_test.jpg", "OpenCVTest/tests/test5/img1_truth.png"),
    ("OpenCVTest/tests/test5/img2_test.jpg", "OpenCVTest/tests/test5/img2_truth.png"),
    ("OpenCVTest/tests/test5/img3_test.jpg", "OpenCVTest/tests/test5/img3_truth.png"),
    ("OpenCVTest/tests/test5/img4_test.jpg", "OpenCVTest/tests/test5/img4_truth.png"),
  ]
  accuracy = []
  images_output = []

  for i, (test_img_name, truth_img_name) in enumerate(images):
    try:
      test_img = cv2.imread(test_img_name, cv2.IMREAD_UNCHANGED)
      result_img = remove_background(test_img)

      truth_img = cv2.imread(truth_img_name, cv2.IMREAD_UNCHANGED)  # BGR and alpha

      images_output.append((test_img, result_img))

      TP, FP, TN, FN = 0, 0, 0, 0
      w = truth_img.shape[1]
      h = truth_img.shape[0]
      for x in range(w):
        for y in range(h):
          alpha = result_img[y, x, 3]  # Alpha channel of the output
          color = truth_img[y, x]

          if color[1] == 255 and color[2] == 0 and color[0] == 0:  # Green pixel
            if alpha >= threshold:
              TP += 1
            else:
              FN += 1
          elif color[2] == 255 and color[1] == 0 and color[0] == 0:  # Red pixel
            if alpha < threshold:
              TN += 1
            else:
              FP += 1

      a = ((TP+TN) / (TP+TN+FN+FP)) * 100
      accuracy.append(a)
      print(f"\tSUBTEST {i+1}: {a}% ({TP}-{FP}/{TN}-{FN})")
    except Exception as e:
      accuracy.append(0.0)
      print(f"\tSUBTEST FAILED: Error")

  worst_accuracy = min(accuracy)
  result = worst_accuracy >= minimum_accuracy
  if result:
    print(f"TEST PASSED (worst case {worst_accuracy}%)")
  else:
    print(f"TEST FAILED (worst case {worst_accuracy}%)")
  
  fig = plt.figure(figsize=(16, 8))
  fig.suptitle("Background Removal Test")
  columns = 4
  rows = 2
  fign = 1
  for i in range(rows):
    for img in images_output:
      ax = fig.add_subplot(rows, columns, fign)
      ax.imshow(cv2.cvtColor(img[i], cv2.COLOR_BGRA2RGBA))  # Correcting for display
      fign += 1

  return result

    

def tests():
  tests = [
    test_face_detection,      # Test 1: Face Detect
    test_edge_detection,      # Test 2: Edge Detect
    test_contour_detect,      # Test 3: Contour Detect
    test_optimisation,        # Test 4: Optimisation
    test_background_removal, # Test 5: Background Removal
  ]
  results = [test() for test in tests]
  
  count = len([result for result in results if result])
  
  print(f'\n\nRESULTS:\n{count} tests passed, {len(tests)-count} tests failed')
  print(f'OVERALL RESULT: {"Pass" if count == len(tests) else "Fail"}')
  
  plt.show()

if __name__ == "__main__":
    tests()
