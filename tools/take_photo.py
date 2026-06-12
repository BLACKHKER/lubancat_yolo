"""
@Author  :Zhao
@Date    :2026/6/2 18:15
@File    :take_photo.py
@Description: TODO
@Version 1.0 
"""
import cv2

FIRST_FRAME = 60

# 相机参数配置
cap = cv2.VideoCapture(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

for _ in range(FIRST_FRAME):
    cap.read()

# 读一帧
ret, frame = cap.read()

# 检查帧是否正确读取
if ret:
    cv2.imshow("Frame", frame)
    cv2.imwrite("photo.jpg", frame)
    print("土拍你保存")
    cv2.destroyAllWindows()
else:
    print("无法读取摄像头")

# 释放摄像头资源
cap.release()
