import cv2
import numpy as np

# Parameters
expected_corners = 3
filename = "photo_carrefour1.jpg"

# Open image and binarize it
img = cv2.imread(filename)
gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
blur = cv2.blur(gray, (8, 8))
# Fita ~170-250, fundo <= ~153 (com reflexos). 124 deixava passar os reflexos.
ret, thresh1 = cv2.threshold(blur, 160, 255, cv2.THRESH_BINARY)

# Morphological processing
# Kernel = linha horizontal 1x300: so sobrevive uma faixa horizontal com >= 300 px.
# Reta simples -> mascara vazia -> 0 cantos ; cruzamento -> faixa horizontal -> varios cantos.
kernel = np.ones((1, 300), np.uint8)
eroded_mask = cv2.erode(thresh1, kernel, iterations=1)
dilated_mask = cv2.dilate(eroded_mask, kernel, iterations=1)

# Corner detection
dilated_mask = np.float32(dilated_mask)
corners = cv2.goodFeaturesToTrack(dilated_mask, 10, 0.1, 30)

# Results information
# goodFeaturesToTrack devolve None quando a mascara esta vazia (sem cruzamento)
corners = np.intp(corners) if corners is not None else []
if len(corners) >= expected_corners:
    print("Intersection !")
else:
    print("No intersection")
for i in corners:
    x, y = i.ravel()
    print(x, y)
    cv2.circle(img, (int(x), int(y)), 8, (0, 0, 255), -1)
cv2.imwrite("out_test.png", img)

cv2.imshow("Corners", img)
cv2.waitKey(0)                # fecha com qualquer tecla
cv2.destroyAllWindows()
