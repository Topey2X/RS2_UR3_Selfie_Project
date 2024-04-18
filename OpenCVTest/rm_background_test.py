## Copied from https://www.geeksforgeeks.org/how-to-remove-the-background-from-an-image-using-python/

# Importing Required Modules 
import rembg
import cv2

# Setup RemBG model session
model_name = "u2net_human_seg" # Pretty good. Specifically trained for humans
# model_name = "isnet-general-use" # Better at hair but had a weird artefact. Possibly ruins the outline.
model_session = rembg.new_session(model_name=model_name)

# Store path of the image in the variable input_path 
input_path = 'OpenCVTest/assets/aiden.jpg' 

# Store path of the output image in the variable output_path 
output_path = 'outputs/bg-removed.png'

# Processing the image 
# input = Image.open(input_path) 
input = cv2.imread(input_path)

# Removing the background from the given Image 
output = rembg.remove(input, session=model_session, bgcolor=(255,255,255,255))

#Saving the image in the given path 
# output.save(output_path) 
cv2.imwrite(output_path, output)
