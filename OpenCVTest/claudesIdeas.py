import numpy as np
import cv2

# Sample contour
contour = np.array([[[181,291]], [[179,293]], [[178,293]], [[177,294]], [[176,294]], [[175,295]], [[174,295]], [[173,296]], [[172,296]], [[171,297]], [[169,297]], [[167,299]], [[164,299]], [[163,300]], [[162,300]], [[161,299]], [[162,300]], [[161,301]], [[162,300]], [[163,300]], [[164,299]], [[167,299]], [[168,298]], [[169,298]], [[170,297]], [[171,297]], [[172,296]], [[173,296]], [[174,295]], [[175,295]], [[176,294]], [[177,294]], [[178,293]], [[179,293]], [[180,292]], [[181,292]], [[182,291]]])

# Flatten the contour to [x, y] coordinates
contour = contour.reshape((-1, 2))

# Fit a polynomial curve to the contour
degree = 3  # Degree of the polynomial curve
x = contour[:, 0]
y = contour[:, 1]
polynomial_coeffs = np.polyfit(x, y, degree)

# Generate smooth curve points
t = np.linspace(x.min(), x.max(), num=100)
smooth_curve = np.poly1d(polynomial_coeffs)(t)

# Plot the original contour and the smooth curve
import matplotlib.pyplot as plt
plt.figure(figsize=(10, 6))
plt.plot(x, y, 'ro', label='Original Contour')
plt.plot(t, smooth_curve, 'b-', label='Smooth Curve')
plt.legend()
plt.show()