import cv2
from math import hypot, floor, degrees, atan2

def convert_img_to_grayscale(img) -> any:
  return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

def identify_faces(img_gray) -> any:
  faceCascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
  faces = faceCascade.detectMultiScale(
    img_gray,
    scaleFactor=1.3,
    minNeighbors=3,
    minSize=(30, 30)
  )
  
  if len(faces) == 0:
    raise 'No valid faces found.'
  
  ## Find biggest face
  biggest_face = [0, None]
  img_found_faces = img_gray.copy()
  for face in faces:
    (x, y, w, h) = face
    cv2.rectangle(img_found_faces, (x,y), (x+w,y+h), (0, 255, 0), 2)
    size = hypot(w,h)
    if size > biggest_face[0]:
      biggest_face = [size, face]
  
  return biggest_face[1]

def crop_img_to_face(img_gray, face) -> any:
  perc_increase : float = 0.35

  # Crop to biggest face, with a 35% increased area
  (x, y, w, h) = face
  w_increase : int = floor(w * perc_increase)
  h_increase : int = floor(h * perc_increase)
  crop_img = img_gray[y-h_increase:y+h+h_increase, x-w_increase:x+w+w_increase]
  
  return crop_img

def extract_edges_from_face(img) -> any:
  blurred = cv2.GaussianBlur(img, (5 , 5), 0)
  # images.append(blurred)
  edges = cv2.Canny(blurred, 80, 130)
  
  return edges

def extract_contours_from_edges(edges) -> any:
  contours, hierarchy = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_TC89_KCOS)
  
  return contours

def simplify_and_split_contours(contours, max_dir_change=90, max_elements_per_array=20, nth_element_simplify=1):  
  def calculate_angle(p1, p2):
      """Calculate the angle between two points from the horizontal."""
      dx = p2[0] - p1[0]
      dy = p2[1] - p1[1]
      return degrees(atan2(dy, dx))

  def split_on_direction_change(contour, max_dir_change):
      """Split contour array if direction change exceeds max_dir_change degrees."""
      splits = []
      current = [contour[0]]  # Initialize with the first point

      for i in range(1, len(contour) - 1):
          angle1 = calculate_angle(contour[i-1][0], contour[i][0])
          angle2 = calculate_angle(contour[i][0], contour[i+1][0])
          angle_change = abs(angle2 - angle1)
          
          # Normalize the angle change to be within 180 degrees
          angle_change = min(angle_change, 360 - angle_change)
          
          if angle_change > max_dir_change:
              # Add the current point to both parts of the split
              current.append(contour[i])
              splits.append(current)
              current = [contour[i]]
          else:
              current.append(contour[i])
      
      # Add the last segment
      current.append(contour[-1])
      splits.append(current)
      
      return splits

  new_contours = []

  # Process original contours
  for contour in contours:
      selected = contour[::nth_element_simplify]
      for i in range(0, len(selected), max_elements_per_array):
          split_array = selected[i:i + max_elements_per_array]
          new_contours.append(split_array)

  processed_contours = []

  # Split contours based on direction change
  for contour in new_contours:
      split_contours = split_on_direction_change(contour, max_dir_change)
      processed_contours.extend(split_contours)
   
  return processed_contours

def eliminate_negligent_contours(contours, min_contour_len=7):
  # ?? Do some fancy checking of the physical length of the contour
  return [contour for contour in contours if len(contour) >= min_contour_len]
