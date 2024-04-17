import cv2
import matplotlib.pyplot as plt
import numpy as np
import subpixel_edges

img = cv2.imread("assets/easy.jpg")
img_gray = (cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)).astype(float)
edges = subpixel_edges.subpixel_edges(img_gray, 10, 1, 2)

fig = plt.figure(figsize=(16, 8))
fign = 1
columns = 3
rows = 1
fig.add_subplot(rows, columns, fign)
fign += 1
plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
plt.axis('equal')

fig.add_subplot(rows, columns, fign)
fign += 1
plt.quiver(edges.x, -edges.y, edges.nx, -edges.ny, color='b', capstyle='round', scale=40)
plt.axis('equal')

fig.add_subplot(rows, columns, fign)
fign += 1
plt.plot(edges.x, -edges.y, 'go', markersize=1)
plt.axis('equal')

# plt.quiver(edges.x-seg/2*edges.ny, edges.y+seg/2*edges.nx, seg*edges.ny, -seg*edges.nx, scale = 40, color = 'r')
# plt.quiver(edges.x, edges.y, edges.nx, -edges.ny, color='b', capstyle='round')
# plt.plot(edges.x, edges.y, 'go-', linewidth=2, markersize=4)
plt.show()