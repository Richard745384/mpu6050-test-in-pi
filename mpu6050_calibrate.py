#!/usr/bin/env python3
"""
MPU6050 校准脚本
────────────────
使用方法：
  1. 将 MPU6050 水平放在桌上，保持静止
  2. 运行：python3 mpu6050_calibrate.py
  3. 等待 ~5 秒采集完成，结果自动保存到 calibration.json
  4. 之后运行 mpu6050_reader.py，它会自动加载校准值

原理：
  在已知静态姿态（水平）下，加速度计/陀螺仪的理论输出为固定值，
  实测值与理论值之差即为 bias（零偏），记录下来后每帧减掉即可。

  期望值（水平静置）：
    Ax=0g, Ay=0g, Az=+1g
    Gx=0°/s, Gy=0°/s, Gz=0°/s
"""

import smbus
import time
import json
import os
import math

# ── MPU6050 寄存器 ─────────────────────────────────────────────────────────────
MPU6050_ADDR  = 0x68
PWR_MGMT_1    = 0x6B
SMPLRT_DIV    = 0x19
CONFIG        = 0x1A
GYRO_CONFIG   = 0x1B
ACCEL_CONFIG  = 0x1C
INT_ENABLE    = 0x38
ACCEL_XOUT_H  = 0x3B
GYRO_XOUT_H   = 0x43

ACCEL_RANGE   = 0   # ±2g
GYRO_RANGE    = 0   # ±250°/s
ACCEL_SCALE   = [16384.0, 8192.0, 4096.0, 2048.0]
GYRO_SCALE    = [131.0, 65.5, 32.8, 16.4]

CALIB_FILE    = os.path.join(os.path.dirname(__file__), "calibration.json")
NUM_SAMPLES   = 500   # 采集帧数（10Hz → ~5 秒）


class MPU6050Raw:
    """最小化的 MPU6050 读取类，仅用于校准"""
    def __init__(self, bus_num=1, address=MPU6050_ADDR):
        self.bus     = smbus.SMBus(bus_num)
        self.address = address
        self._init()

    def _write(self, reg, val):
        self.bus.write_byte_data(self.address, reg, val)

    def _read_word(self, reg):
        hi  = self.bus.read_byte_data(self.address, reg)
        lo  = self.bus.read_byte_data(self.address, reg + 1)
        val = (hi << 8) | lo
        return val - 65536 if val >= 0x8000 else val

    def _init(self):
        self._write(PWR_MGMT_1,   0x00)
        self._write(SMPLRT_DIV,   0x07)
        self._write(CONFIG,       0x06)  # 最大滤波（校准时降噪）
        self._write(GYRO_CONFIG,  GYRO_RANGE  << 3)
        self._write(ACCEL_CONFIG, ACCEL_RANGE << 3)
        self._write(INT_ENABLE,   0x01)
        time.sleep(0.1)  # 等待稳定

    def read_frame(self):
        """读取一帧 (ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps)"""
        as_ = ACCEL_SCALE[ACCEL_RANGE]
        gs  = GYRO_SCALE[GYRO_RANGE]
        return {
            "ax": self._read_word(ACCEL_XOUT_H)     / as_,
            "ay": self._read_word(ACCEL_XOUT_H + 2) / as_,
            "az": self._read_word(ACCEL_XOUT_H + 4) / as_,
            "gx": self._read_word(GYRO_XOUT_H)      / gs,
            "gy": self._read_word(GYRO_XOUT_H + 2)  / gs,
            "gz": self._read_word(GYRO_XOUT_H + 4)  / gs,
        }


def collect_samples(mpu, n):
    """采集 n 帧，带进度条"""
    samples = []
    print(f"\n  采集中 [{'.' * 50}]", end="", flush=True)
    bar_step = n // 50
    for i in range(n):
        samples.append(mpu.read_frame())
        if (i + 1) % bar_step == 0:
            filled = (i + 1) // bar_step
            print(f"\r  采集中 [{'█' * filled}{'.' * (50 - filled)}] {i+1}/{n}", end="", flush=True)
        time.sleep(0.01)  # 100Hz 采集（比显示快，数据量更充分）
    print()
    return samples


def compute_bias(samples):
    """计算各轴均值，与理论值对比得到 bias"""
    keys = ["ax", "ay", "az", "gx", "gy", "gz"]
    means = {k: sum(s[k] for s in samples) / len(samples) for k in keys}

    # 理论期望值（水平静置）: az = +1g，其余 = 0
    expected = {"ax": 0.0, "ay": 0.0, "az": 1.0,
                "gx": 0.0, "gy": 0.0, "gz": 0.0}

    bias = {k: means[k] - expected[k] for k in keys}
    return means, bias


def print_report(means, bias, samples):
    """打印校准结果报告"""
    keys = ["ax", "ay", "az", "gx", "gy", "gz"]
    units = {"ax": "g", "ay": "g", "az": "g", "gx": "°/s", "gy": "°/s", "gz": "°/s"}

    # 计算标准差（衡量噪声）
    stds = {}
    for k in keys:
        vals = [s[k] for s in samples]
        mean = means[k]
        stds[k] = math.sqrt(sum((v - mean) ** 2 for v in vals) / len(vals))

    print("\n" + "=" * 60)
    print("  校准结果报告")
    print("=" * 60)
    print(f"  {'轴':<8} {'均值':>10} {'Bias':>10} {'噪声(σ)':>10} {'单位':>6}")
    print("-" * 60)
    for k in keys:
        print(f"  {k:<8} {means[k]:>10.4f} {bias[k]:>10.4f} {stds[k]:>10.4f} {units[k]:>6}")
    print("=" * 60)

    # 健康评估
    print("\n  健康评估：")
    accel_mag = math.sqrt(means["ax"]**2 + means["ay"]**2 + means["az"]**2)
    print(f"  合加速度 = {accel_mag:.4f} g  (理论 = 1.0000 g，差距 {abs(accel_mag - 1.0)*100:.2f}%)")

    gyro_noise_ok = all(stds[k] < 1.0 for k in ["gx", "gy", "gz"])
    print(f"  陀螺仪噪声 {'✅ 正常 (<1°/s)' if gyro_noise_ok else '⚠️  偏大，校准期间可能有轻微抖动'}")

    if abs(accel_mag - 1.0) > 0.05:
        print("  ⚠️  合加速度偏差 >5%，请确认传感器放置水平且完全静止")
    else:
        print("  ✅ 合加速度正常")


def main():
    print("=" * 60)
    print("  MPU6050 校准程序")
    print("=" * 60)
    print(f"\n  请将 MPU6050 水平放置在桌面上，保持静止")
    print(f"  将采集 {NUM_SAMPLES} 帧（约 {NUM_SAMPLES * 0.01:.0f} 秒）")
    print(f"  结果将保存到：{CALIB_FILE}")

    input("\n  准备好后按 Enter 开始...")

    try:
        mpu = MPU6050Raw()
    except Exception as e:
        print(f"\n  ❌ 连接 MPU6050 失败: {e}")
        print("  请检查：sudo dtoverlay i2c1，以及接线是否正确")
        return

    print(f"\n  正在采集，请保持静止...")
    samples = collect_samples(mpu, NUM_SAMPLES)

    means, bias = compute_bias(samples)
    print_report(means, bias, samples)

    # 保存到 JSON
    result = {
        "bias": bias,
        "means": means,
        "num_samples": len(samples),
        "timestamp": time.strftime("%Y-%m-%d %Human:%M:%S"),
        "accel_range": ACCEL_RANGE,
        "gyro_range":  GYRO_RANGE,
    }
    with open(CALIB_FILE, "w") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)

    print(f"\n  ✅ 校准完成！Bias 已保存到 {CALIB_FILE}")
    print("  现在可以运行：python3 mpu6050_reader.py\n")


if __name__ == "__main__":
    main()
