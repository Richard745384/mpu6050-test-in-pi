#!/usr/bin/env python3
"""
树莓派拍照脚本
支持：官方 Pi Camera 模块 / USB 摄像头
用法：python3 take_photo.py [输出文件名]
"""

import sys
import os
from datetime import datetime


def get_output_filename():
    """获取输出文件名，默认使用时间戳"""
    if len(sys.argv) > 1:
        filename = sys.argv[1]
        if not filename.endswith(('.jpg', '.jpeg', '.png')):
            filename += '.jpg'
        return filename
    return f"photo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"


def capture_with_picamera2(output_file):
    """使用 picamera2 拍照（适用于树莓派官方摄像头，新版系统）"""
    from picamera2 import Picamera2
    import time

    print("📷 使用 Picamera2 拍照...")
    cam = Picamera2()
    config = cam.create_still_configuration(main={"size": (1920, 1080)})
    cam.configure(config)
    cam.start()
    time.sleep(2)  # 等待自动曝光稳定
    cam.capture_file(output_file)
    cam.stop()
    print(f"✅ 照片已保存：{os.path.abspath(output_file)}")


def capture_with_picamera(output_file):
    """使用 picamera 拍照（旧版 picamera 库）"""
    import picamera
    import time

    print("📷 使用 picamera 拍照...")
    with picamera.PiCamera() as camera:
        camera.resolution = (1920, 1080)
        camera.start_preview()
        time.sleep(2)  # 等待自动曝光稳定
        camera.capture(output_file)
    print(f"✅ 照片已保存：{os.path.abspath(output_file)}")


def capture_with_opencv(output_file, device_index=0):
    """使用 OpenCV 拍照（适用于 USB 摄像头）"""
    import cv2
    import time

    print(f"📷 使用 OpenCV 拍照（设备: /dev/video{device_index}）...")
    cap = cv2.VideoCapture(device_index)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开摄像头设备 {device_index}")

    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    time.sleep(1)  # 等待摄像头初始化

    ret, frame = cap.read()
    cap.release()

    if not ret:
        raise RuntimeError("无法从摄像头读取画面")

    cv2.imwrite(output_file, frame)
    print(f"✅ 照片已保存：{os.path.abspath(output_file)}")


def main():
    output_file = get_output_filename()
    print(f"🎯 输出文件：{output_file}")

    # 按优先级尝试不同的拍照方式
    methods = [
        ("picamera2",  capture_with_picamera2),
        ("picamera",   capture_with_picamera),
        ("opencv",     capture_with_opencv),
    ]

    for name, method in methods:
        try:
            method(output_file)
            return  # 成功则退出
        except ImportError:
            print(f"⚠️  {name} 未安装，尝试下一种方式...")
        except Exception as e:
            print(f"❌ {name} 失败：{e}，尝试下一种方式...")

    print("\n❌ 所有方式均失败，请按以下提示安装依赖：")
    print("  Pi Camera 模块：sudo apt install -y python3-picamera2")
    print("  USB 摄像头：   pip3 install opencv-python")
    sys.exit(1)


if __name__ == "__main__":
    main()
