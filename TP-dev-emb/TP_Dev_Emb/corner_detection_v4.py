import os
import sys
import time

import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import Transform

# Parameters
expected_corners = 3          # um "+" tem 4 cantos côncavos; 3 tolera perder um
win = 25                      # janela (px) onde se mede a fração de branco em volta de cada canto
concave_min, concave_max = 0.60, 0.85


def detect_crossing_corners(img):
    # 1) Binarização: Otsu escolhe o limiar sozinho (robusto a variações de iluminação)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)   # imread e Picamera2 'RGB888' entregam BGR
    blur = cv2.blur(gray, (8, 8))
    ret, thresh1 = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # 2) Abertura: remove tudo menor que 17x17 (reflexos, ruído).
    #    Tem que ficar abaixo da espessura da fita mais fina (~33 px).
    kernel = np.ones((17, 17), np.uint8)
    eroded_mask = cv2.erode(thresh1, kernel, iterations=1)
    dilated_mask = cv2.dilate(eroded_mask, kernel, iterations=1)

    # 3) Candidatos: pedimos muitos cantos, porque o goodFeaturesToTrack também
    #    marca degraus das bordas inclinadas e as pontas das fitas.
    corners = cv2.goodFeaturesToTrack(np.float32(dilated_mask), 40, 0.05, 15)
    if corners is None:
        return np.zeros((0, 2)), ret
    corners = corners.reshape(-1, 2)

    # 4) Classificação pela fração de branco numa janela win x win centrada no canto:
    #      ~0.75 -> canto CÔNCAVO (os 4 cantos internos do cruzamento)
    #      ~0.50 -> borda reta (degraus, falsos cantos)
    #      ~0.25 -> canto convexo (pontas da fita)
    frac = cv2.blur(dilated_mask.astype(np.float32) / 255, (win, win))
    h, w = gray.shape
    m = win // 2 + 1
    corners = corners[(corners[:, 0] >= m) & (corners[:, 0] < w - m) &
                      (corners[:, 1] >= m) & (corners[:, 1] < h - m)]
    f = frac[corners[:, 1].round().astype(int), corners[:, 0].round().astype(int)]
    return corners[(f > concave_min) & (f < concave_max)], ret

# libcamera debug logging level: 0 for all, 1 for info, 2 for warning, 3 for error and 4 for fatal error
os.environ["LIBCAMERA_LOG_LEVELS"] = "3"

# Sem janela quando rodando via SSH sem tela: python3 corner_detection_v4.py --no-display
show = "--no-display" not in sys.argv

# Mesma configuração do perception2.py: 1280x1024 é a resolução para a qual
# os parâmetros (kernel 17x17, win 25) foram ajustados.
camera = Picamera2()
config = camera.create_preview_configuration({"size": (1280, 1024), "format": 'RGB888'},
                                             raw=camera.sensor_modes[7],
                                             transform=Transform(hflip=1, vflip=1))
camera.configure(config)
camera.start()
time.sleep(1)

try:
    while True:
        t0 = time.perf_counter()
        img = camera.capture_array("main")
        corners, otsu = detect_crossing_corners(img)
        dt = time.perf_counter() - t0

        if len(corners) >= expected_corners:
            cx, cy = corners.mean(axis=0)
            print(f"Intersection ! {len(corners)} cantos, centro em ({cx:.0f}, {cy:.0f})  "
                  f"Otsu={otsu:.0f}  {dt * 1000:.0f} ms ({1 / dt:.1f} fps)")
        else:
            print(f"No intersection ({len(corners)} cantos)  Otsu={otsu:.0f}  "
                  f"{dt * 1000:.0f} ms ({1 / dt:.1f} fps)")

        if show:
            if len(corners) >= expected_corners:
                cv2.drawMarker(img, (int(cx), int(cy)), (0, 255, 0), cv2.MARKER_CROSS, 30, 3)
            for x, y in corners:
                cv2.circle(img, (int(x), int(y)), 8, (0, 0, 255), -1)
            cv2.imshow("Corners", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):   # 'q' para sair
                break
except KeyboardInterrupt:                           # Ctrl+C para sair
    pass
finally:
    camera.stop()
    cv2.destroyAllWindows()
