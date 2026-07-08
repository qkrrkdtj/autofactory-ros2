# diag_back.py — cam_back 실제 프레임 한 장 저장
import cv2
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.opencv.camera_opencv import OpenCVCamera

cam = OpenCVCamera(OpenCVCameraConfig(
    index_or_path='/dev/cam_back', fps=30, width=640, height=480))
cam.connect()
img = cam.read()                      # RGB
img = img[:, :, ::-1].copy()          # BGR
cv2.imwrite('canon_back.png', img)
print("저장: canon_back.png  shape:", img.shape)
cam.disconnect()