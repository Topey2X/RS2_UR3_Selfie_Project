import matplotlib.pyplot as plt
import csv

def read_csv_to_array(file_path):
    coordinates = []
    with open(file_path, 'r') as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')  
        next(csv_reader)  
        for row in csv_reader:
            x, y, z = map(float, row)  
            coordinates.append((x, y, z))  
    return coordinates

file_path = 'Final_Waypoints_Robot.csv'

points = read_csv_to_array(file_path)

x_points = [p[0] for p in points]
y_points = [p[1] for p in points]
z_points = [p[2] for p in points]

plt.scatter(x_points, y_points, color='blue', s=1)
for i, (x, y) in enumerate(zip(x_points[1:], y_points[1:]), start=1):
    plt.text(x, y, '', fontsize=1, ha='right')

plt.xlabel('X')
plt.ylabel('Y')
plt.xlim(0, 210)  
plt.ylim(0, 297)  
plt.pause(1)

for i in range(len(x_points) - 1):
    if z_points[i] <= 1:
        plt.plot([x_points[i], x_points[i + 1]], [y_points[i], y_points[i + 1]], linestyle='-', marker='o', color='blue', markersize=1, linewidth=1)
        plt.draw()
        plt.pause(0.1)

plt.plot([x_points[-1], x_points[1]], [y_points[-1], y_points[1]], linestyle='-', marker='o', color='blue', markersize=1, linewidth=1)
plt.draw()

plt.show()
