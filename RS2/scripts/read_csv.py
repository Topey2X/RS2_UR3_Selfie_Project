import csv
from collections import namedtuple
import pandas as pd

# Define a namedtuple to represent a Point
Point = namedtuple('Point', ['x', 'y', 'z'])

# Read the CSV file into a DataFrame
df = pd.read_csv('Final_Waypoints_Robot.csv', engine='python')

# Convert the DataFrame to a list of Point namedtuples
points = [Point(*row) for row in df.values]

# Now you can access the points like:
for point in points:
    print(f"Point: ({point.x}, {point.y}, {point.z})")
    