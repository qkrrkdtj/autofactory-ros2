# grab_tray_omx1.py  — omx1 wrist 트레이 프레임 저장용
import numpy as np
import cv2
from lerobot.robots.utils import make_robot_from_config
from lerobot.robots.omx_follower import OmxFollowerConfig
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from config import OMX_CONFIGS

cfg = OMX_CONFIGS["omx1"]
tray_key = cfg.get("tray_camera_key", "observation.images.wrist")

cams = {k: OpenCVCameraConfig(index_or_path=c["index_or_path"], fps=c["fps"],
                              width=c["width"], height=c["height"])
        for k, c in cfg["cameras"].items()}
robot = make_robot_from_config(OmxFollowerConfig(port=cfg["robot_port"], cameras=cams))
robot.connect()

obs = robot.get_observation()
img = obs.get(tray_key)
if img is None:
    print("키 없음. 가능 키:", [k for k in obs if "image" in k])
    robot.disconnect(); raise SystemExit

if hasattr(img, "detach"):
    img = img.detach().cpu().numpy()
img = np.asarray(img)
if img.ndim == 3 and img.shape[0] in (1, 3) and img.shape[2] not in (1, 3):
    img = np.transpose(img, (1, 2, 0))            # CHW -> HWC
if img.dtype != np.uint8:
    img = ((img * 255).clip(0, 255).astype(np.uint8)
           if img.max() <= 1.0 else img.astype(np.uint8))

bgr = img[:, :, ::-1].copy()                       # RGB -> BGR (cv2 저장용)
cv2.imwrite("tray_omx1.png", bgr)
print("저장됨: tray_omx1.png", bgr.shape)
robot.disconnect()
