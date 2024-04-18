import cv2
import matplotlib.pyplot as plt
import numpy as np
import subpixel_edges

np.bool = np.bool_
np.int = int

img = cv2.imread("OpenCVTest/assets/easy.jpg")
img_gray = (cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)).astype(float)
edges = subpixel_edges.subpixel_edges(img_gray, 10, 1, 2)

# fig = plt.figure(figsize=(16, 8))
# fign = 1
# columns = 4
# rows = 1
# fig.add_subplot(rows, columns, fign)
# fign += 1
# plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
# plt.axis('equal')

# fig.add_subplot(rows, columns, fign)
# fign += 1
# plt.quiver(edges.x, -edges.y, edges.nx, -edges.ny, color='b', capstyle='round', scale=40)
# plt.axis('equal')

# fig.add_subplot(rows, columns, fign)
# fign += 1
# plt.plot(edges.x, -edges.y, 'go', markersize=1)
# plt.axis('equal')

# plt.quiver(edges.x-seg/2*edges.ny, edges.y+seg/2*edges.nx, seg*edges.ny, -seg*edges.nx, scale = 40, color = 'r')
# plt.quiver(edges.x, edges.y, edges.nx, -edges.ny, color='b', capstyle='round')
# plt.plot(edges.x, edges.y, 'go-', linewidth=2, markersize=4)
# plt.show()

import numpy as np
from scipy.spatial.distance import cdist
from sklearn.cluster import DBSCAN

def normalize_vectors(vectors):
    norms = np.linalg.norm(vectors, axis=1, keepdims=True)
    return vectors / norms

def are_vectors_aligned(v1, v2, angle_threshold=np.cos(np.pi/10)):  # 18 degrees threshold
    return np.dot(v1, v2) >= angle_threshold

def fit_lines_to_clusters(points, labels):
    unique_labels = np.unique(labels)
    lines = []
    for label in unique_labels:
        if label != -1:  # -1 is for noise points
            cluster_points = points[labels == label]
            pca = np.linalg.svd(cluster_points - cluster_points.mean(axis=0))
            direction = pca[0][:, 0]
            point_on_line = cluster_points.mean(axis=0)
            lines.append((point_on_line, direction))
    return lines

def convert_points_to_lines(edges, distance_threshold=10, angle_threshold=np.cos(np.pi/10)):
    points = np.column_stack((edges.x, edges.y))
    normals = np.column_stack((edges.nx, edges.ny))
    n_points = len(points)
    distances = cdist(points, points)
    normals = normalize_vectors(normals)
    adjacency_matrix = np.zeros((n_points, n_points), dtype=bool)
    
    for i in range(n_points):
        for j in range(i + 1, n_points):
            if (distances[i, j] < distance_threshold and 
                are_vectors_aligned(normals[i], normals[j], angle_threshold)):
                adjacency_matrix[i, j] = adjacency_matrix[j, i] = True

    clustering = DBSCAN(eps=distance_threshold, min_samples=2, metric='precomputed')
    labels = clustering.fit_predict(1 - adjacency_matrix.astype(int))
    return fit_lines_to_clusters(points, labels)
  
lines = convert_points_to_lines(edges)

# print(lines)

# plt.show()

# Plotting
fig, ax = plt.subplots()
ax.scatter(edges.x, edges.y, color='blue', label='Points')

# Define how far to extend the lines
line_length = 5

for point, direction in lines:
    start_point = point - direction * line_length
    end_point = point + direction * line_length
    ax.plot([start_point[0], end_point[0]], [start_point[1], end_point[1]], 'r-')

ax.set_title('Lines fitted to points')
ax.set_xlabel('X coordinate')
ax.set_ylabel('Y coordinate')
ax.grid(True)
ax.axis('equal')
ax.legend()
plt.show()
