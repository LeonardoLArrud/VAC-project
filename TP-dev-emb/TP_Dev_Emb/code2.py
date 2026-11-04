import cv2
import numpy as np

# Read an image and print its dimensions
image = cv2.imread("photo_test.jpg")
im2 = cv2.imread("photo_test.jpg")
h, w = image.shape[:2]
print(w, h)


# Convert to gray scale image
gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

# Blur and threshold
blur = cv2.blur(gray, (1, 1))
ret, thresh = cv2.threshold(blur, 124, 255, cv2.THRESH_BINARY)


# Remove noise with morphological processing
kernel = np.ones((4, 4), np.uint8)
eroded_mask = cv2.erode(thresh, kernel, iterations=1)
closed_mask = cv2.dilate(eroded_mask, kernel, iterations=1)


# Find the different contours
contours, hierarchy = cv2.findContours(
    closed_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
)
# Sort by area and draw the output image
contours = sorted(contours, key=cv2.contourArea, reverse=True)
im2 = cv2.drawContours(im2, contours, -1, (0, 255, 0), 2)

cv2.imwrite("out_test.png", im2)


# Compute barycenter
print(len(contours))
if len(contours) > 0:
    M = cv2.moments(contours[0])
    # Centroid
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    print(f"Centroid of the biggest area: ({cx}, {cy})")
else:
    print("No Centroid Found")


cv2.imshow("origin", image)
cv2.imshow("blur", blur)
cv2.imshow("gray", gray)
cv2.imshow("thresh", thresh)
cv2.imshow("closed", closed_mask)
cv2.imshow("output", im2)
cv2.waitKey(0)
cv2.destroyAllWindows()
