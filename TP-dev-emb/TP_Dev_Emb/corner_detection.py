import cv2
import numpy as np


def corner_detection(filename):
    # Parameters
    expected_corners = 3
    filename = "photo_carrefour1.jpg"

    # Open image and binarize it
    img = cv2.imread(filename)
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    blur = cv2.blur(gray, (8, 8))
    ret, thresh1 = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # Morphological processing
    kernel = np.ones((9, 9), np.uint8)
    eroded_mask = cv2.erode(thresh1, kernel, iterations=1)
    dilated_mask = cv2.dilate(eroded_mask, kernel, iterations=1)

    # Corner detection
    dilated_mask = np.float32(dilated_mask)
    corners = cv2.goodFeaturesToTrack(
        dilated_mask, 4, 0.1, 25, blockSize=9, useHarrisDetector=True, k=0.15
    )

    # Results information
    corners = np.intp(corners)
    if len(corners) >= expected_corners:
        print("Intersection !")
    for i in corners:
        x, y = i.ravel()
        print(x, y)
        cv2.circle(img, (x, y), 3, 255, -1)
    cv2.imwrite("out_test.png", img)

    cv2.imshow("Corners", img)
    cv2.waitKey(0)                # fecha com qualquer tecla
    cv2.destroyAllWindows()


if __name__ == "__main__":
    corner_detection("photo_carrefour1.jpg")
