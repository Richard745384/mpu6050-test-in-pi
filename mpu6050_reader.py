#!/usr/bin/env python3
"""
MPU6050 六轴传感器读取脚本 (加速度计 + 陀螺仪)
适用于树莓派，通过 I2C 接口通信

接线方式：
  MPU6050  ->  树莓派
  VCC      ->  3.3V (Pin 1)
  GND      ->  GND  (Pin 6)
  SCL      ->  SCL  (Pin 5 / GPIO3)
  SDA      ->  SDA  (Pin 3 / GPIO2)
  AD0      ->  GND  (I2C 地址 0x68) 或 3.3V (地址 0x69)

安装依赖：
  sudo apt-get install -y python3-smbus i2c-tools
  sudo raspi-config  -> Interface Options -> I2C -> Enable
"""

import smbus
import time
import math
import json
import os

# ── MPU6050 寄存器地址 ─────────────────────────────────────────────────────────
MPU6050_ADDR      = 0x68   # AD0 接 GND；若接 3.3V 则改为 0x69

PWR_MGMT_1        = 0x6B
SMPLRT_DIV        = 0x19
CONFIG            = 0x1A
GYRO_CONFIG       = 0x1B
ACCEL_CONFIG      = 0x1C
INT_ENABLE        = 0x38

ACCEL_XOUT_H      = 0x3B
GYRO_XOUT_H       = 0x43
TEMP_OUT_H        = 0x41

# ── 量程配置 ───────────────────────────────────────────────────────────────────
# 加速度量程: 0=±2g, 1=±4g, 2=±8g, 3=±16g
ACCEL_RANGE = 0
ACCEL_SCALE = [16384.0, 8192.0, 4096.0, 2048.0]

# 陀螺仪量程: 0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s
GYRO_RANGE  = 0
GYRO_SCALE  = [131.0, 65.5, 32.8, 16.4]

# ── 校准文件路径 ───────────────────────────────────────────────────────────────
CALIB_FILE = os.path.join(os.path.dirname(__file__), "calibration.json")


def load_calibration():
    """加载校准文件，返回 bias 字典；文件不存在则返回全零 bias"""
    zero_bias = {"ax": 0.0, "ay": 0.0, "az": 0.0,
                 "gx": 0.0, "gy": 0.0, "gz": 0.0}
    if not os.path.exists(CALIB_FILE):
        print("[校准] 未找到 calibration.json，使用原始数据（无校准）")
        print("[校准] 提示：先运行 python3 mpu6050_calibrate.py 生成校准文件")
        return zero_bias
    try:
        with open(CALIB_FILE) as f:
            data = json.load(f)
        bias = data.get("bias", zero_bias)
        ts   = data.get("timestamp", "未知")
        print(f"[校准] 已加载校准文件（生成时间：{ts}）")
        print(f"[校准] Bias → ax={bias['ax']:+.4f}g  ay={bias['ay']:+.4f}g  az={bias['az']:+.4f}g  "
              f"gx={bias['gx']:+.2f}°/s  gy={bias['gy']:+.2f}°/s  gz={bias['gz']:+.2f}°/s")
        return bias
    except Exception as e:
        print(f"[校准] 读取校准文件失败: {e}，使用原始数据")
        return zero_bias


class MPU6050:
    def __init__(self, bus_num: int = 1, address: int = MPU6050_ADDR, bias: dict = None):
        self.bus     = smbus.SMBus(bus_num)
        self.address = address
        self.bias    = bias or {"ax": 0.0, "ay": 0.0, "az": 0.0,
                                "gx": 0.0, "gy": 0.0, "gz": 0.0}
        self._init_sensor()

    def _write(self, reg: int, value: int):
        self.bus.write_byte_data(self.address, reg, value)

    def _read_word_2c(self, reg: int) -> int:
        """读取两字节有符号整数（大端，补码）"""
        high = self.bus.read_byte_data(self.address, reg)
        low  = self.bus.read_byte_data(self.address, reg + 1)
        val  = (high << 8) | low
        return val - 65536 if val >= 0x8000 else val

    def _init_sensor(self):
        self._write(PWR_MGMT_1,   0x00)           # 唤醒传感器
        self._write(SMPLRT_DIV,   0x07)           # 采样率分频
        self._write(CONFIG,       0x00)           # 不滤波
        self._write(GYRO_CONFIG,  GYRO_RANGE  << 3)
        self._write(ACCEL_CONFIG, ACCEL_RANGE << 3)
        self._write(INT_ENABLE,   0x01)
        print(f"[MPU6050] 初始化完成，I2C 地址: 0x{self.address:02X}")

    # ── 原始数据读取 ──────────────────────────────────────────────────────────
    def read_accel_raw(self) -> tuple:
        x = self._read_word_2c(ACCEL_XOUT_H)
        y = self._read_word_2c(ACCEL_XOUT_H + 2)
        z = self._read_word_2c(ACCEL_XOUT_H + 4)
        return x, y, z

    def read_gyro_raw(self) -> tuple:
        x = self._read_word_2c(GYRO_XOUT_H)
        y = self._read_word_2c(GYRO_XOUT_H + 2)
        z = self._read_word_2c(GYRO_XOUT_H + 4)
        return x, y, z

    def read_temp_raw(self) -> int:
        return self._read_word_2c(TEMP_OUT_H)

    # ── 工程单位换算（含校准修正）────────────────────────────────────────────
    def read_accel(self) -> dict:
        """返回校准后的加速度 (单位: g)"""
        scale = ACCEL_SCALE[ACCEL_RANGE]
        x, y, z = self.read_accel_raw()
        return {
            "ax": x / scale - self.bias["ax"],
            "ay": y / scale - self.bias["ay"],
            "az": z / scale - self.bias["az"],
        }

    def read_gyro(self) -> dict:
        """返回校准后的角速度 (单位: °/s)"""
        scale = GYRO_SCALE[GYRO_RANGE]
        x, y, z = self.read_gyro_raw()
        return {
            "gx": x / scale - self.bias["gx"],
            "gy": y / scale - self.bias["gy"],
            "gz": z / scale - self.bias["gz"],
        }

    def read_temperature(self) -> float:
        """返回芯片温度 (单位: ℃)"""
        return self.read_temp_raw() / 340.0 + 36.53

    def read_all(self) -> dict:
        """一次性读取所有数据（校准后）"""
        data = {}
        data.update(self.read_accel())
        data.update(self.read_gyro())
        data["temp"] = self.read_temperature()
        return data

    # ── 姿态角估算（基于加速度计，静态） ──────────────────────────────────────
    def get_tilt_angles(self) -> dict:
        """
        利用重力向量估算俯仰角 (pitch) 和滚转角 (roll)，单位：度
        注意：仅在静止或低动态时准确，不含偏航角 (yaw)
        """
        a = self.read_accel()
        ax, ay, az = a["ax"], a["ay"], a["az"]
        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))
        roll  = math.degrees(math.atan2(ay, az))
        return {"pitch": pitch, "roll": roll}


# ── 主程序：持续打印传感器数据 ────────────────────────────────────────────────
def main():
    bias = load_calibration()
    print()

    mpu = MPU6050(bus_num=1, bias=bias)
    print("\n按 Ctrl+C 退出\n")
    print(f"{'时间':>10}  {'Ax(g)':>8} {'Ay(g)':>8} {'Az(g)':>8}  "
          f"{'Gx(°/s)':>9} {'Gy(°/s)':>9} {'Gz(°/s)':>9}  "
          f"{'Pitch(°)':>9} {'Roll(°)':>8}  {'Temp(℃)':>8}")
    print("-" * 100)

    try:
        while True:
            d  = mpu.read_all()
            tl = mpu.get_tilt_angles()
            ts = time.strftime("%H:%M:%S")
            print(
                f"{ts:>10}  "
                f"{d['ax']:>8.3f} {d['ay']:>8.3f} {d['az']:>8.3f}  "
                f"{d['gx']:>9.2f} {d['gy']:>9.2f} {d['gz']:>9.2f}  "
                f"{tl['pitch']:>9.2f} {tl['roll']:>8.2f}  "
                f"{d['temp']:>8.2f}"
            )
            time.sleep(0.1)   # 10 Hz 刷新
    except KeyboardInterrupt:
        print("\n已退出。")


if __name__ == "__main__":
    main()
