import sys
import time
from multiprocessing import Process

import cv2

from corner_detection_v3 import detect_crossing_corners

filename = sys.argv[1] if len(sys.argv) > 1 else "photo_carrefour1.jpg"


def intersection(img):
    corners, _ = detect_crossing_corners(img)
    print("Intersection !" if len(corners) >= 3 else "No intersection")


def barycentre(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(
        cv2.blur(gray, (8, 8)), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    M = cv2.moments(thresh, binaryImage=True)
    if M["m00"] > 0:
        print(
            f"Barycentre de la piste : ({M['m10'] / M['m00']:.0f}, {M['m01'] / M['m00']:.0f})"
        )
    else:
        print("Pas de piste visible")


if __name__ == "__main__":
    img = cv2.imread(filename)
    starttime = time.perf_counter()

    p1 = Process(target=intersection, args=(img,))
    p2 = Process(target=barycentre, args=(img,))
    p1.start()
    p2.start()
    p1.join()
    p2.join()

    print(time.perf_counter() - starttime)
