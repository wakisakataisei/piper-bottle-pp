"""ArUco マーカー（DICT_4X4_50, ID 0）の画像を作る → data/aruco_4x4_id0.png
印刷済みの A4 PDF は docs/aruco_id0_50mm_A4.pdf（黒い正方形の一辺 50 mm）。
印刷後に一辺をノギスで測り、その値を 05_handeye.py に渡す（実機では 51.0 mm だった）。
"""
import numpy as np
import cv2
import _bootstrap  # noqa: F401
from piper_pp.config import data_path

d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
m = cv2.aruco.generateImageMarker(d, 0, 600)
img = np.full((800, 800), 255, np.uint8)
img[100:700, 100:700] = m
cv2.imwrite(str(data_path("aruco_4x4_id0.png")), img)
print("data/aruco_4x4_id0.png を作成（ID 0）。黒い正方形の一辺が 50〜60 mm になるよう印刷")
