from picamera2 import Picamera2
from libcamera import Transform
import time
import cv2
import os

# libcamera debug logging level: 0 for all, 1 for info, 2 for warning, 3 for error and 4 for fatal error
os.environ["LIBCAMERA_LOG_LEVELS"]="3"

# Configure camera for (1280x1024) mode and suitable format
camera = Picamera2();
config = camera.create_preview_configuration({"size": (1280,1024),"format": 'RGB888'},raw=camera.sensor_modes[7],transform=Transform(hflip=1,vflip=1));
camera.configure(config);


# Start camera
camera.start()
time.sleep(1)

starttime=time.perf_counter()

#Generate directly a numpy array with the image 
array = camera.capture_array("main")
print(time.perf_counter() - starttime)
print(array.shape)
cv2.imwrite('test2.jpg', array)
camera.stop()
