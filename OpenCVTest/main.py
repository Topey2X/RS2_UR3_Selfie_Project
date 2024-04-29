from lib_img_processing import *
from typing import List, Tuple
from cv2.typing import MatLike

DEBUG = True

def process_image(img : MatLike) -> any:
  add_plot("Image", "Webcam Image", img)
  
  # Step 4: Convert image to grayscale
  img_gray = convert_img_to_grayscale(img)
  
  # Step 5: Identify faces
  face = identify_faces(img_gray)
  
  # Step 6: Crop to Face
  cropped_img = crop_img_to_face(img, face, perc_increase=0.35)
  add_plot("Image", "Cropped to Face", cropped_img)
  
  # Step 7: Remove Background from Face
  no_bg_img = remove_background(cropped_img)
  add_plot("Image", "No Background", no_bg_img)
  
  # Step 6: Normalise Image Brightness and Contrast
  normalised_img = normalise_brightness_contrast(no_bg_img, target_contrast=1.4)
  add_plot("Image", "Normalised", normalised_img)
  
  # Step 8: Extract Edges from Face
  edges_img = extract_edges_from_face(normalised_img, upper=80, lower=130)
  add_plot("Image", "Edges", edges_img)
  
  # Step 9: Extract Contours from Edges
  contours, _ = extract_contours_from_edges(edges_img)
  add_plot("Contours", "Raw Contours", contours)
  
  # Step 10: Optimise Contours #1: Simplify and Split
  contours = simplify_and_split_contours(contours)
  
  # Step 11: Optimise Contours #2: Eliminate Negligent Contours
  contours = eliminate_negligent_contours(contours)
  
  # Step 12: Smooth Contours
  smoothed_contours = smooth_contours(contours)
  add_plot("Contours", "Processed Contours", smoothed_contours)
  
  # Step 13: Publish ROS message
  return smoothed_contours

plots : List[Tuple[str, str, any]] = []
def add_plot(type : str, title : str, data : any) -> None:
  if not DEBUG:
    return
  
  plots.append((type, title, data))

def show_to_screen() -> None:  
  import matplotlib.pyplot as plt
  
  fig = plt.figure(figsize=(16, 8))
  columns = 4
  rows = ((len(plots)-1) // columns)+1
  
  for i, (type, title, data) in enumerate(plots):
    ax = fig.add_subplot(rows, columns, i+1)
    if type == "Image":
      plt.imshow(cv2.cvtColor(data, cv2.COLOR_BGR2RGB))
    elif type == "GreyImage":
      plt.imshow(data, cmap = plt.cm.gray)
    elif type == "Contours":
      for contour in data:
        points = np.array(contour).reshape(-1, 2)
        plt.plot(points[:, 0], -points[:, 1])
      ax.set_aspect('equal')
    ax.set_title(title)

  plt.show()
  
def save_contours(contours):  
  from os import mkdir  
  output_file_path = "outputs/output.txt"
  # mkdir("outputs")
  # Exporting the array to the file
  with open(output_file_path, "w") as file:
      for contour in contours:
          file.write(str(contour) + '\n')
  

if __name__ == '__main__':
  if DEBUG:
    img = cv2.imread("OpenCVTest/assets/two_faces.jpg")
    save_contours(process_image(img))
    show_to_screen()
  else:
    from OpenCVTest.lib_ros_functions import ROSNODE_img_processor
    rosnode = ROSNODE_img_processor(process_image)
    rosnode.run()
  