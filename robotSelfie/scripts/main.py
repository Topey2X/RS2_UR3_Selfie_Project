#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from robotSelfie.msg import ContourList, Contour
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Bool
import cv2

# GUI
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# from img_processing import *
# from ros_functions import *
from cv2.typing import MatLike
from cv_bridge import CvBridge
import rembg
from math import hypot, floor, degrees, atan2
import numpy as np
from scipy.interpolate import splprep, splev
from typing import List, Tuple

DEBUG = False


class Processor_Node:
    plots: List[Tuple[str, str, any]] = []
    root = None # Tkinter root
    image_subscriber = None
    contour_publisher = None
    finished_subscriber = None
    has_published = False
    capture_button = None
    use_button = None
    reset_button = None
    capture_next_image = False
    capture_button_sprite = None
    use_button_sprite = None  
    reset_button_sprite = None  
    blank_img = None
    figure_canvas = None
  
    # GUI
    def create_gui(self):
        self.root = tk.Tk()
        self.root.title("RobotSelfie")

        self.capture_button_sprite = tk.PhotoImage(file="/home/darren2004/git/RS2_UR3_Selfie_Project/robotSelfie/scripts/icons/capture.png")
        self.use_button_sprite = tk.PhotoImage(file="/home/darren2004/git/RS2_UR3_Selfie_Project/robotSelfie/scripts/icons/tick.png")
        self.reset_button_sprite = tk.PhotoImage(file="/home/darren2004/git/RS2_UR3_Selfie_Project/robotSelfie/scripts/icons/reset.png")

        self.capture_button = tk.Button(
            self.root,
            compound=tk.BOTTOM,
            image=self.capture_button_sprite,
            text="Capture",
            command=self.capture_image,
            bg="white",
            fg="black",
            font=("Helvetica", 14, "bold"),
            padx=20,
            pady=10,
            relief="raised",
        )
        self.capture_button.grid(row=0,column=0,pady=10)

        self.use_button = tk.Button(
            self.root,
            compound=tk.BOTTOM,
            image=self.use_button_sprite,
            text="Use Image",
            command=self.use_image,
            state=tk.DISABLED,
            bg="gray",
            fg="white",
            font=("Helvetica", 12),
            padx=15,
            pady=8,
        )
        self.use_button.grid(row=1,column=0,pady=5)
        
        self.reset_button = tk.Button(
            self.root,
            compound=tk.BOTTOM,
            image=self.reset_button_sprite,
            text="Reset GUI",
            command=self.reset_gui,
            state=tk.NORMAL,
            bg="gray",
            fg="white",
            font=("Helvetica", 12),
            padx=15,
            pady=8,
        )
        self.reset_button.grid(row=2,column=0,pady=5)
        
        # Generate blank image initially
        self.frame = tk.Frame(self.root)
        self.blank_img = np.zeros((720,1280,3), dtype=np.uint8)
        self.update_image(self.blank_img)
        self.frame.grid(row=0, column=1, rowspan=3, padx=10, pady=10)

    def enable_buttons(self, capture=False, use=False):
        self.capture_button.config(state=(tk.NORMAL if capture else tk.DISABLED))
        self.use_button.config(state=(tk.NORMAL if use else tk.DISABLED))

    # Capture Image
    def capture_image(self):
        self.enable_buttons()
        self.capture_next_image = True
        rospy.loginfo("Capturing image...")

    # Use image if it is good
    def use_image(self):
        self.enable_buttons()
        rospy.loginfo("Using captured image...")
        self.process_captured_image()
        self.update_image(plots=True)
        
    def reset_gui(self):
        rospy.loginfo("Resetting GUI...")
        self.enable_buttons(capture=True, use=False)
        self.update_image(self.blank_img)
        self.plots = []
        self.has_published = False
        self.captured_image = None
        self.capture_next_image = False
        # Publish a message to indicate GUI reset and new data availability
        self.reset_publisher.publish(True)

    # Processed the image after the user agrees with the result
    def process_captured_image(self):
        if self.captured_image is not None:
            # Process the captured image
            try:
                processed_result = self.process_image(self.captured_image)
            except:
                rospy.logwarn("Error processing image.")
                messagebox.showinfo("Error", "Error processing image.")
                self.update_image(self.blank_img)
                self.enable_buttons(capture=True, use=False)
                return
            # Publish the processed result
            self.publish_contours(processed_result)
        else:
            messagebox.showinfo("Error", "No image captured.")

    def process_image(self, img: MatLike) -> any:
        # self.add_plot("Image", "Webcam Image", img)

        # Step 1: Convert image to grayscale
        img_gray = self.convert_img_to_grayscale(img)

        # Step 2: Identify faces
        face, _ = self.identify_faces(img_gray)

        # Step 3: Crop to Face
        cropped_img = self.crop_img_to_face(img, face, perc_increase=0.35)
        self.add_plot("Image", "Cropped to Face", cropped_img)

        # Step 4: Remove Background from Face
        no_bg_img = self.remove_background(cropped_img)
        self.add_plot("Image", "No Background", no_bg_img)

        # Step 5: Normalise Image Brightness and Contrast
        normalised_img = self.normalise_brightness_contrast(
            no_bg_img, target_contrast=1.4
        )
        self.add_plot("Image", "Normalised", normalised_img)

        # Step 6: Extract Edges from Face
        edges_img = self.extract_edges_from_face(normalised_img, upper=80, lower=130)
        self.add_plot("Image", "Edges", edges_img)

        # Step 7: Extract Contours from Edges
        contours, _ = self.extract_contours_from_edges(edges_img)
        self.add_plot("Contours", "Raw Contours", contours)

        # Step 8: Optimise Contours #1: Simplify and Split
        contours = self.simplify_and_split_contours(contours)

        # Step 9: Optimise Contours #2: Eliminate Negligent Contours
        contours = self.eliminate_negligent_contours(contours)

        # Step 10: Smooth Contours
        smoothed_contours = self.smooth_contours(contours)
        self.add_plot("Contours", "Processed Contours", smoothed_contours)

        # Step 11: Publish ROS message (completed elsewhere)
        return smoothed_contours

    def add_plot(self, type: str, title: str, data: any) -> None:
        self.plots.append((type, title, data))

    def show_to_screen(self) -> None:
        fig = plt.figure(figsize=(16, 8))
        columns = 4
        rows = ((len(self.plots) - 1) // columns) + 1

        for i, (type, title, data) in enumerate(self.plots):
            ax = fig.add_subplot(rows, columns, i + 1)
            if type == "Image":
                plt.imshow(cv2.cvtColor(data, cv2.COLOR_BGR2RGB))
            elif type == "GreyImage":
                plt.imshow(data, cmap=plt.cm.gray)
            elif type == "Contours":
                for contour in data:
                    points = np.array(contour).reshape(-1, 2)
                    plt.plot(points[:, 0], -points[:, 1])
                ax.set_aspect("equal")
            ax.set_title(title)

        plt.show()

    def save_contours(self, contours):
        import os

        output_dir = "outputs"
        output_file_path = os.path.join(output_dir, "output.txt")

        # Create the 'outputs' directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)

        # Write the contours to the file
        with open(output_file_path, "w") as file:
            for contour in contours:
                file.write(str(contour) + "\n")

    def convert_img_to_grayscale(self, img) -> any:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    def identify_faces(self, img_gray) -> any:
        faceCascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
        )
        faces = faceCascade.detectMultiScale(
            img_gray, scaleFactor=1.3, minNeighbors=3, minSize=(30, 30)
        )

        if len(faces) == 0:
            raise Exception("No valid faces found.")

        ## Find biggest face
        biggest_face = [0, None]
        img_found_faces = img_gray.copy()
        for face in faces:
            (x, y, w, h) = face
            cv2.rectangle(img_found_faces, (x, y), (x + w, y + h), (0, 255, 0), 2)
            size = hypot(w, h)
            if size > biggest_face[0]:
                biggest_face = [size, face]

        return biggest_face[1], img_found_faces

    def crop_img_to_face(self, img, face, perc_increase) -> MatLike:
        # Crop to biggest face, with a 35% increased area
        (x, y, w, h) = face
        w_increase: int = floor(w * perc_increase)
        h_increase: int = floor(h * perc_increase)
        crop_img = img[
            y - h_increase : y + h + h_increase, x - w_increase : x + w + w_increase
        ]

        return crop_img

    def remove_background(self, img) -> MatLike:
        # Setup RemBG model session
        # model_name = "u2net_human_seg" # Pretty good. Specifically trained for humans
        model_name = "isnet-general-use"  # Better at hair but had a weird artefact. Possibly ruins the outline.
        model_session = rembg.new_session(model_name=model_name)

        # no_bg_img = rembg.remove(img, session=model_session, bgcolor=(100,100,100,255)) # Replace background with white.
        no_bg_img = rembg.remove(img, session=model_session)  # Remove background

        return no_bg_img

    def normalise_brightness_contrast(self, img, target_contrast) -> MatLike:
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
        alpha_factor = alpha_factor[
            ..., None
        ]  # Expand dims to broadcast in multiplication
        blended_rgb = (1 - alpha_factor) * rgb + alpha_factor * adjusted_rgb

        # Combine the blended RGB with the original alpha channel
        adjusted_image = np.dstack((blended_rgb, alpha)).astype(np.uint8)

        return adjusted_image

    def extract_edges_from_face(self, img, upper, lower) -> MatLike:
        blurred = cv2.GaussianBlur(img, (5, 5), 0)
        # images.append(blurred)
        edges = cv2.Canny(blurred, upper, lower)

        return edges

    def extract_contours_from_edges(self, edges) -> tuple:
        contours, hierarchy = cv2.findContours(
            edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        # contours, hierarchy = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_TC89_KCOS)

        return contours, hierarchy

    def simplify_and_split_contours(
        self,
        contours,
        max_dir_change=90,
        max_elements_per_array=20,
        nth_element_simplify=1,
    ) -> list:
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
                angle1 = calculate_angle(contour[i - 1][0], contour[i][0])
                angle2 = calculate_angle(contour[i][0], contour[i + 1][0])
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
                split_array = selected[i : i + max_elements_per_array]
                new_contours.append(split_array)

        processed_contours = []

        # Split contours based on direction change
        for contour in new_contours:
            split_contours = split_on_direction_change(contour, max_dir_change)
            processed_contours.extend(split_contours)

        return processed_contours

    def eliminate_negligent_contours(self, contours, min_contour_len=7) -> list:
        # ?? Do some fancy checking of the physical length of the contour
        return [contour for contour in contours if len(contour) >= min_contour_len]

    def smooth_contours(self, contours, points_per_contour=None) -> list:
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
            y = (
                np.array(contour).reshape(-1, 2).T
            )  # Transpose to have 2 rows: one for x and one for y

            # Fit a spline to the 3D curve (index, x, y)
            tck, u = splprep([x, y[0], y[1]], s=3, k=3)

            # Evaluate the spline across the original parameter range
            if points_per_contour is None:
                new_points = splev(u, tck)
            else:
                sample_points = np.linspace(0, 1, points_per_contour)
                new_points = splev(sample_points, tck)

            # Extract the smoothed curve, ignoring the first dimension (index)
            smoothed_contour = np.vstack(
                new_points[1:]
            ).T  # Transpose back to original shape

            return smoothed_contour

        return [smooth_contour(contour) for contour in contours]

    def publish_contours(self, contours):
        # Build the message
        contour_list_msg = ContourList()
        for contour in contours:
            contour_msg = Contour()
            contour_msg.points = [Point(x, y, 0.0) for x, y in contour]
            contour_list_msg.contours.append(contour_msg)
        # Publish the message
        self.contour_publisher.publish(contour_list_msg)
        rospy.loginfo("Contours published.")

    # Callback function when an image is received
    def image_callback(self, data):
        if self.capture_next_image:
            rospy.loginfo("Image received")
            # Receive the image and convert to a cv2 image
            self.captured_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
            self.capture_next_image = False
            self.enable_buttons(capture=True, use=True)
            self.update_image(self.captured_image)
            rospy.loginfo("Image saved, ready for confirmation ...")
            
    def update_image(self, img=None, plots=False):
        for widget in self.frame.winfo_children():
            widget.destroy()
            
        if plots and len(self.plots) >= 6:
            # Display the last 6 plots

            fig = plt.figure(figsize=(16, 8))
            columns = 3
            rows = 2

            for i, (type, title, data) in enumerate(self.plots[-6:]):
                ax = fig.add_subplot(rows, columns, i + 1)
                if type == "Image":
                    ax.imshow(cv2.cvtColor(data, cv2.COLOR_BGR2RGB))
                elif type == "GreyImage":
                    ax.imshow(data, cmap=plt.cm.gray)
                elif type == "Contours":
                    for contour in data:
                        points = np.array(contour).reshape(-1, 2)
                        ax.plot(points[:, 0], -points[:, 1])
                    ax.set_aspect("equal")
                ax.set_title(title)

            # Create a canvas to display the figure
            canvas = FigureCanvasTkAgg(fig, master=self.frame)
            canvas.draw()
            self.figure_canvas = canvas.get_tk_widget()
            self.figure_canvas.grid(row=0, column=0, padx=10, pady=10)

        else:
            if img is None:
                img = self.blank_img
            # Display a single image
            cv2_img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(cv2_img_rgb)
            tk_img = ImageTk.PhotoImage(image=pil_img)
            label = tk.Label(self.frame, image=tk_img)
            label.image = tk_img
            label.grid(row=0, column=0, padx=10, pady=10)
            
    def finished_callback(self, data):
        self.has_published = False
        self.captured_image = None
        self.update_image(self.blank_img)
        self.enable_buttons(capture=True, use=False)     

    def main(self):
        """
        Main for use in ROS environment
        """
        self.contour_publisher = rospy.Publisher("contours", ContourList, queue_size=10)
        
        # Create a publisher for reset messages
        self.reset_publisher = rospy.Publisher("reset_gui", Bool, queue_size=10)
        
        # Initialize the ROS node
        rospy.init_node("image_processor")
        rospy.sleep(1)
        # Create a subscriber to listen to the webcam images
        self.image_subscriber = rospy.Subscriber(
            "/usb_cam/image_raw", ImageMsg, self.image_callback
        )
        self.finished_subscriber = rospy.Subscriber(
            "finished", Bool, self.finished_callback
        )

        # Initialize the captured_image attribute
        self.captured_image = None
        
        # Create the GUI
        self.create_gui()
        self.enable_buttons(capture=True, use=False)


    ## Static image main
    def debug_main(
        self,
    ):
        """
        Main for use with static image, saving only.
        """
        # img = cv2.imread("OpenCVTest/assets/two_faces.jpg")
        img = cv2.imread(
            "/home/darren2004/git/RS2_UR3_Selfie_Project/robotSelfie/scripts/OpenCVTest/assets/easy.jpg"
        )
        contours = self.process_image(img)
        self.save_contours(contours)
        self.show_to_screen()

    def __init__(self, DEBUG):
        self.DEBUG = DEBUG
        if DEBUG:
            self.debug_main()
            self.root.mainloop()
        else:
            self.main()
            self.root.mainloop()


if __name__ == "__main__":
    node = Processor_Node(DEBUG)