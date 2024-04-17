import cv2

def subscribe_to_webcam():
  pass

def capture_image_from_webcam() -> any:
  inputImagePath = "assets/easy.jpg"
  img = cv2.imread(inputImagePath)
  return img

def publish_to_ROS():
  pass
