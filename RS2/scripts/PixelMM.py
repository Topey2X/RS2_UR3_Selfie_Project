import csv
import pandas as pd

# Image, paper dimensions, and file path information
image_width = 450
image_height = 400
paper_width_mm = 297
paper_height_mm = 210
input_file = "/home/jacob/git/RS2_UR3_Selfie_Project/RS2/scripts/Waypoints.csv"
output_file = '/home/jacob/git/RS2_UR3_Selfie_Project/RS2/scripts/Waypoints_mm.csv'
scaled_output_file = '/home/jacob/git/RS2_UR3_Selfie_Project/RS2/scripts/Waypoints_mm_scaled.csv'

def pixel_to_mm(input_file, output_file, image_width, image_height, paper_width_mm, paper_height_mm):
    """
    Convert point coordinates from pixels to mm and save to a new file.
    """
    # Calculate scale
    scale_x = paper_width_mm / image_width
    scale_y = paper_height_mm / image_height

    # Read CSV file, convert points, and write to a new file
    with open(input_file, 'r') as file_in, open(output_file, 'w', newline='') as file_out:
        reader = csv.reader(file_in)
        writer = csv.writer(file_out)
        writer.writerow(['x', 'y', 'z'])  # Add header
        for row in reader:
            if not row:  # Skip empty rows
                continue
            x_pixels, y_pixels, z = map(float, row)
            x_mm = x_pixels * scale_x 
            y_mm = y_pixels * scale_y
            z_int = int(z)  # Convert z to integer
            writer.writerow([f'{x_mm:.4f}', f'{y_mm:.4f}', z_int])  # Save with 4 decimal places and z as an integer

    # Read the new file and print values
    with open(output_file, 'r') as file_out:
        reader = csv.reader(file_out)
        next(reader)  # Skip header
        row_num = 2
        for row in reader:
            x_mm, y_mm = map(float, row[:2])  # Read x and y as float
            z = int(row[2])  # Read z as integer
            # print(f"Hàng số {row_num}: giá trị x_mm là {x_mm:.4f}, giá trị y_mm là {y_mm:.4f} và giá trị z là {z}")
            print(f"Row number {row_num}: x_mm value is {x_mm:.4f}, y_mm value is {y_mm:.4f} and z value is {z}")
            row_num += 1


# Perform conversion and print results
pixel_to_mm(input_file, output_file, image_width, image_height, paper_width_mm, paper_height_mm)
print("Completed!")

# Read the CSV file and check columns
waypoints_mm = pd.read_csv(output_file)
print("Columns in DataFrame:", waypoints_mm.columns)

# Reduce all values in Waypoints_mm.csv by 10%
waypoints_mm[['x', 'y']] *= 0.9
waypoints_mm['z'] = waypoints_mm['z'].astype(int)  # Ensure column z is integer


# Save the reduced results with 4 decimal places
waypoints_mm.to_csv(scaled_output_file, index=False, float_format='%.4f')

print("Reduced values in Waypoints_mm.csv by 10% and saved to Waypoints_mm_scaled.csv with 4 decimal places")

# Print the reduced values with 4 decimal places
with open(scaled_output_file, 'r') as file_out:
    reader = csv.reader(file_out)
    next(reader)  # Skip header
    row_num = 2
    for row in reader:
        x_mm, y_mm = map(float, row[:2])  # Read x and y as float
        z = int(row[2])  # Read z as integer
        # print(f"Hàng số {row_num}: giá trị x_mm là {x_mm:.4f}, giá trị y_mm là {y_mm:.4f} và giá trị z là {z}")
        print(f"Row number {row_num}: x_mm value is {x_mm:.4f}, y_mm value is {y_mm:.4f} and z value is {z}")
        row_num += 1

# Check if x, y coordinates are within the size of A4 paper
x_in_paper_range = (waypoints_mm['x'] >= 0) & (waypoints_mm['x'] <= 210)
y_in_paper_range = (waypoints_mm['y'] >= 0) & (waypoints_mm['y'] <= 297)
all_points_in_paper = x_in_paper_range.all() & y_in_paper_range.all()


# Find and print the minimum and maximum values of x and y
x_min, x_max = waypoints_mm['x'].min(), waypoints_mm['x'].max()
y_min, y_max = waypoints_mm['y'].min(), waypoints_mm['y'].max()

print(f"x values range from {x_min:.4f} to {x_max:.4f}")
print(f"y values range from {y_min:.4f} to {y_max:.4f}")

print("All points are within the paper size:", all_points_in_paper)
if not all_points_in_paper:
    out_of_paper_points = waypoints_mm[~x_in_paper_range | ~y_in_paper_range]
    print("Points outside the paper size:", out_of_paper_points)
