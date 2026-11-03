import cv2

# Read an image and print its dimensions
image = cv2.imread("image_test.png")
print(image.shape)

# Display the image
cv2.imshow("Exemple", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
