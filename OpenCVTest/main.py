from ROS_functions import * 
from img_processing_functions import *

def main():
  # Step X: Subscribe to ROS Webcam
  subscribe_to_webcam()
  
  # Step X: Wait for 'Go' to be pressed (..GUI?)
  input('Enter to start')
  
  # Step X: Capture image from webcam
  img = capture_image_from_webcam()
  
  # Step X: Convert image to grayscale
  img_gray = convert_img_to_grayscale(img)
  
  # Step X: Identify faces
  faces = identify_faces(img_gray)
  
  # Step X: Crop to Face
  cropped_img = crop_img_to_face(img_gray, faces)
  
  # Step X: Extract Edges from Face
  edges_img = extract_edges_from_face(cropped_img)
  
  # Step X: Extract Contours from Edges
  contours = extract_contours_from_edges(edges_img)
  
  # Step X: Optimise Contours #1: Simplify and Split
  contours = simplify_and_split_contours(contours)
  
  # Step X: Optimise Contours #2: Eliminate Negligent Contours
  

if __name__ == '__main__':
  main()