import cv2
from cv2.typing import MatLike
import rembg
from math import hypot, floor, degrees, atan2
import numpy as np
from scipy.interpolate import splprep, splev

def convert_img_to_grayscale(img) -> any:
  return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

def identify_faces(img_gray) -> MatLike:
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

def crop_img_to_face(img, face, perc_increase) -> MatLike:
  # Crop to biggest face, with a 35% increased area
  (x, y, w, h) = face
  w_increase : int = floor(w * perc_increase)
  h_increase : int = floor(h * perc_increase)
  crop_img = img[y-h_increase:y+h+h_increase, x-w_increase:x+w+w_increase]
  
  return crop_img

def remove_background(img) -> MatLike:
  # Setup RemBG model session
  # model_name = "u2net_human_seg" # Pretty good. Specifically trained for humans
  model_name = "isnet-general-use" # Better at hair but had a weird artefact. Possibly ruins the outline.
  model_session = rembg.new_session(model_name=model_name)
  
  # no_bg_img = rembg.remove(img, session=model_session, bgcolor=(100,100,100,255)) # Replace background with white.
  no_bg_img = rembg.remove(img, session=model_session) # Remove background
  
  return no_bg_img

def normalise_brightness_contrast(img, target_contrast) -> MatLike:
    rgb, alpha = img[..., :3], img[..., 3]

    # Convert RGB to grayscale for brightness analysis
    # gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

    # Calculate the average brightness of the face only
    # mask = alpha > 0  # Mask where alpha is not zero
    # masked_gray = gray[mask]
    # average_brightness = np.mean(masked_gray)
    
    # Calculate beta - adjustment for brightness
    # beta = target_brightness - average_brightness
    beta = 0
    
    # Calculate alpha - adjustment for contrast
    # alpha_adj = target_contrast if average_brightness < target_brightness else 1 / target_contrast
    alpha_adj = target_contrast
    
    # Prepare the adjusted RGB image
    adjusted_rgb = cv2.convertScaleAbs(rgb, alpha=alpha_adj, beta=beta)
    
    # Blend the adjusted RGB image with the original based on alpha values
    # Pixels with no transparency receive full change, those with full transparency remain unchanged
    alpha_factor = alpha / 255.0  # Normalize alpha to range [0, 1]
    alpha_factor = alpha_factor[..., None]  # Expand dims to broadcast in multiplication
    blended_rgb = (1 - alpha_factor) * rgb + alpha_factor * adjusted_rgb
    
    # Combine the blended RGB with the original alpha channel
    adjusted_image = np.dstack((blended_rgb, alpha)).astype(np.uint8)
    
    return adjusted_image

def extract_edges_from_face(img, upper, lower) -> MatLike:
  blurred = cv2.GaussianBlur(img, (5 , 5), 0)
  # images.append(blurred)
  edges = cv2.Canny(blurred, upper, lower)
  
  return edges

def extract_contours_from_edges(edges) -> tuple:
  contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 
  # contours, hierarchy = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_TC89_KCOS) 
  
  return contours, hierarchy

def simplify_and_split_contours(contours, max_dir_change=90, max_elements_per_array=20, nth_element_simplify=1) -> list:  
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

def eliminate_negligent_contours(contours, min_contour_len=7) -> list:
  # ?? Do some fancy checking of the physical length of the contour
  return [contour for contour in contours if len(contour) >= min_contour_len]

def smooth_contours(contours, points_per_contour=None) -> list:
  def smooth_contour(contour):
    """
    Smooth a given contour by approximating it with a 3rd degree polynomial curve in 3D,
    using the index as the x-value, and then project it back to a 2D curve.
    
    Parameters:
    - contour: The input contour, a list of [x, y] points.
    
    Returns:
    - smoothed_contour: The smoothed 2D contour as a list of [x, y] points.
    """
    # Convert contour to an appropriate format and include index as x value
    x = np.arange(len(contour))
    y = np.array(contour).reshape(-1, 2).T  # Transpose to have 2 rows: one for x and one for y
    
    # Fit a spline to the 3D curve (index, x, y)
    tck, u = splprep([x, y[0], y[1]], s=3, k=3)
    
    # Evaluate the spline across the original parameter range
    if points_per_contour is None:
      new_points = splev(u, tck)
    else:
      sample_points = np.linspace(0, 1, points_per_contour)
      new_points = splev(sample_points, tck)
    
    # Extract the smoothed curve, ignoring the first dimension (index)
    smoothed_contour = np.vstack(new_points[1:]).T  # Transpose back to original shape
    
    return smoothed_contour
  
  return [smooth_contour(contour) for contour in contours]
