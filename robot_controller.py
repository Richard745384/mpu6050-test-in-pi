#!/usr/bin/env python3
"""
robot_controller.py — MPU6050 实时状态识别 + 控制指令输出
============================================================
功能：
  1. 读取 MPU6050（含校准）
  2. 互补滤波 → Roll / Pitch / Yaw
  3. 状态机 → 12 种机器人状态识别
  4. 控制器 → 根据状态生成速度指令
  5. 终端实时刷新显示

运行：
  python3 robot_controller.py

控制指令输出接口（扩展用）:
  修改 RobotController.send_command() 将指令发给串口/ROS/socket等
"""

import smbus
import time
import math
import json
import os
import signal
from collections import deque

# ═══════════════════════════════════════════════════════════
# 1. MPU6050 驱动（带校准）
# ═══════════════════════════════════════════════════════════
MPU6050_ADDR = 0x68
ACCEL_SCALE  = [16384.0, 8192.0, 4096.0, 2048.0]  # ±2g~±16g
GYRO_SCALE   = [131.0, 65.5, 32.8, 16.4]           # ±250~±2000°/s
ACCEL_RANGE  = 0
GYRO_RANGE   = 0
CALIB_FILE   = os.path.join(os.path.dirname(__file__), "calibration.json")


def load_calibration():
    zero = {"ax": 0.0, "ay": 0.0, "az": 0.0, "gx": 0.0, "gy": 0.0, "gz": 0.0}
    if not os.path.exists(CALIB_FILE):
        print("[校准] 未找到 calibration.json，将使用原始数据")
        return zero
    with open(CALIB_FILE) as f:
        d = json.load(f)
    bias = d.get("bias", zero)
    print(f"[校准] 已加载校准文件  ax={bias['ax']:+.4f}g "
          f"gx={bias['gx']:+.2f}°/s  gy={bias['gy']:+.2f}°/s")
    return bias


class MPU6050Driver:
    def __init__(self, bus=1, addr=MPU6050_ADDR, bias=None):
        self.bus  = smbus.SMBus(bus)
        self.addr = addr
        self.bias = bias or {k: 0.0 for k in ["ax","ay","az","gx","gy","gz"]}
        self._init()

    def _write(self, reg, val):
        self.bus.write_byte_data(self.addr, reg, val)

    def _read2(self, reg):
        hi = self.bus.read_byte_data(self.addr, reg)
        lo = self.bus.read_byte_data(self.addr, reg + 1)
        v  = (hi << 8) | lo
        return v - 65536 if v >= 0x8000 else v

    def _init(self):
        self._write(0x6B, 0x00)   # PWR_MGMT_1: 唤醒
        self._write(0x6B, 0x01)   # 时钟源：PLL with X gyro
        self._write(0x1A, 0x03)   # DLPF: ~44Hz 带宽
        self._write(0x19, 0x13)   # 采样率: 1kHz/(1+19)=50Hz
        self._write(0x1B, GYRO_RANGE  << 3)
        self._write(0x1C, ACCEL_RANGE << 3)
        print(f"[MPU6050] 初始化完成，地址 0x{self.addr:02X}，采样率 50Hz")

    def read(self):
        """读取一帧，返回校准后的 ax/ay/az(g) gx/gy/gz(°/s) temp(℃)"""
        as_ = ACCEL_SCALE[ACCEL_RANGE]
        gs  = GYRO_SCALE[GYRO_RANGE]
        return {
            "ax": self._read2(0x3B) / as_ - self.bias["ax"],
            "ay": self._read2(0x3D) / as_ - self.bias["ay"],
            "az": self._read2(0x3F) / as_ - self.bias["az"],
            "gx": self._read2(0x43) / gs  - self.bias["gx"],
            "gy": self._read2(0x45) / gs  - self.bias["gy"],
            "gz": self._read2(0x47) / gs  - self.bias["gz"],
            "temp": self._read2(0x41) / 340.0 + 36.53,
            "ts_ms": int(time.monotonic() * 1000),
        }


# ═══════════════════════════════════════════════════════════
# 2. 互补滤波器（Roll / Pitch / Yaw）
# ═══════════════════════════════════════════════════════════
class ComplementaryFilter:
    """
    angle = α × (angle + gyro × dt) + (1-α) × accel_angle
    α=0.96: 96% 信任陀螺仪，4% 用加速度计修正漂移
    yaw 只靠陀螺仪积分（无磁力计，长期漂移）
    """
    def __init__(self, alpha=0.96):
        self.alpha = alpha
        self.roll = self.pitch = self.yaw = 0.0
        self._last_ts = None
        self._inited  = False

    def update(self, d):
        ax, ay, az = d["ax"], d["ay"], d["az"]
        gx, gy, gz = d["gx"], d["gy"], d["gz"]
        ts = d["ts_ms"]

        # 第一帧用加速度计初始化
        if not self._inited:
            self.roll  = math.degrees(math.atan2(ay, az))
            self.pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))
            self.yaw   = 0.0
            self._last_ts = ts
            self._inited  = True
            return self.roll, self.pitch, self.yaw

        dt = max((ts - self._last_ts) / 1000.0, 1e-3)
        self._last_ts = ts

        # 陀螺仪积分
        roll_g  = self.roll  + gx * dt
        pitch_g = self.pitch + gy * dt
        yaw_g   = self.yaw   + gz * dt

        # 加速度计估算 roll/pitch
        roll_a  = math.degrees(math.atan2(ay, az))
        pitch_a = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))

        # 互补融合
        self.roll  = self.alpha * roll_g  + (1 - self.alpha) * roll_a
        self.pitch = self.alpha * pitch_g + (1 - self.alpha) * pitch_a
        self.yaw   = yaw_g   # 无磁力计，只积分

        return self.roll, self.pitch, self.yaw


# ═══════════════════════════════════════════════════════════
# 3. 状态机（完整复现 C++ IMUStateEstimator）
# ═══════════════════════════════════════════════════════════
G = 9.80665  # m/s²

# 状态定义
STATIONARY        = "STATIONARY"
MOVING_FORWARD    = "MOVING_FORWARD"
TURNING           = "TURNING"
FALLEN            = "FALLEN"
STEEP_SLOPE       = "STEEP_SLOPE"
ROLLOVER_WARNING  = "ROLLOVER_WARNING"
ROLLOVER          = "ROLLOVER"
FREEFALL          = "FREEFALL"
SENSOR_FAULT      = "SENSOR_FAULT"
SPIN              = "SPIN"
VIOLENT_COLLISION = "VIOLENT_COLLISION"
VIBRATION_ABNORMAL= "VIBRATION_ABNORMAL"
UNKNOWN           = "UNKNOWN"

# 状态对应的终端显示颜色
STATE_COLOR = {
    STATIONARY:         "\033[92m",   # 绿
    MOVING_FORWARD:     "\033[96m",   # 青
    TURNING:            "\033[93m",   # 黄
    FALLEN:             "\033[91m",   # 红
    STEEP_SLOPE:        "\033[93m",   # 黄
    ROLLOVER_WARNING:   "\033[91m",   # 红
    ROLLOVER:           "\033[91m",   # 红
    FREEFALL:           "\033[91m",   # 红
    SENSOR_FAULT:       "\033[91m",   # 红
    SPIN:               "\033[91m",   # 红
    VIOLENT_COLLISION:  "\033[91m",   # 红
    VIBRATION_ABNORMAL: "\033[93m",   # 黄
    UNKNOWN:            "\033[90m",   # 灰
}
RESET = "\033[0m"


class StateEstimator:
    """
    阈值全部可在构造时定制，默认值与 C++ 版一致
    """
    def __init__(self,
                 fall_roll=45, fall_pitch=45,
                 move_accel=0.5,         # m/s²
                 turn_gyro=20,           # °/s
                 slope_min=25, slope_max=120,
                 rollover_warn_min=30, rollover_warn_max=120,
                 rollover_deg=120,
                 freefall_g=0.5,
                 sensor_fault_max=0.01,
                 spin_gyro=200, spin_frames=6,
                 collision_spike=30,     # m/s²
                 vibration_std=1.5,      # m/s²
                 vibration_window=25):
        self.th = dict(
            fall_roll=fall_roll, fall_pitch=fall_pitch,
            move_accel=move_accel, turn_gyro=turn_gyro,
            slope_min=slope_min, slope_max=slope_max,
            rollover_warn_min=rollover_warn_min, rollover_warn_max=rollover_warn_max,
            rollover_deg=rollover_deg, freefall_g=freefall_g,
            sensor_fault_max=sensor_fault_max,
            spin_gyro=spin_gyro, spin_frames=spin_frames,
            collision_spike=collision_spike,
            vibration_std=vibration_std, vibration_window=vibration_window,
        )
        self.state = UNKNOWN
        self._spin_cnt = 0
        self._prev_accel = -1.0
        self._collision_triggered = False
        self._spike_peak = 0.0
        self._accel_win = deque()

    def update(self, d, roll, pitch):
        ax, ay, az = d["ax"] * G, d["ay"] * G, d["az"] * G  # 转为 m/s²
        gx, gy, gz = d["gx"], d["gy"], d["gz"]

        total_accel = math.sqrt(ax**2 + ay**2 + az**2)
        total_g     = total_accel / G
        horiz_accel = math.sqrt(ax**2 + ay**2)
        abs_roll    = abs(roll)
        abs_pitch   = abs(pitch)
        th = self.th

        # 0. 传感器故障
        accel_sum = abs(ax) + abs(ay) + abs(az)
        if accel_sum / G < th["sensor_fault_max"]:
            self._reset_counters()
            self.state = SENSOR_FAULT
            return self.state

        # 1. 自由落体
        if total_g < th["freefall_g"]:
            self._reset_counters()
            self.state = FREEFALL
            return self.state

        # 2. 剧烈碰撞 (spike 检测)
        if self._prev_accel >= 0:
            rise = total_accel - self._prev_accel
            if rise > th["collision_spike"]:
                self._collision_triggered = True
                self._spike_peak = total_accel
            elif self._collision_triggered:
                if total_accel < self._spike_peak - th["collision_spike"]:
                    self._collision_triggered = False
                    self._spike_peak = 0
                    self._prev_accel = total_accel
                    self._spin_cnt = 0
                    self._accel_win.clear()
                    self.state = VIOLENT_COLLISION
                    return self.state
        self._prev_accel = total_accel

        # 3. 翻转
        if abs_roll > th["rollover_deg"] or d["az"] < -1.0:
            self._spin_cnt = 0
            self._accel_win.clear()
            self.state = ROLLOVER
            return self.state

        # 4. 急转 SPIN
        if abs(gz) > th["spin_gyro"]:
            self._spin_cnt += 1
        else:
            self._spin_cnt = 0
        if self._spin_cnt >= th["spin_frames"]:
            self._accel_win.clear()
            self.state = SPIN
            return self.state

        # 5. 异常振动
        self._accel_win.append(total_accel)
        if len(self._accel_win) > th["vibration_window"]:
            self._accel_win.popleft()
        if len(self._accel_win) >= th["vibration_window"]:
            mean = sum(self._accel_win) / len(self._accel_win)
            std  = math.sqrt(sum((v - mean)**2 for v in self._accel_win) / len(self._accel_win))
            if std > th["vibration_std"]:
                self.state = VIBRATION_ABNORMAL
                return self.state

        # 6. 翻滚预警
        if th["rollover_warn_min"] <= abs_roll <= th["rollover_warn_max"]:
            self.state = ROLLOVER_WARNING
            return self.state

        # 7. 陡坡
        if th["slope_min"] <= abs_pitch <= th["slope_max"]:
            self.state = STEEP_SLOPE
            return self.state

        # 8. 跌倒
        if abs_roll > th["fall_roll"] or abs_pitch > th["fall_pitch"]:
            self.state = FALLEN
            return self.state

        # 9. 转弯
        if abs(gz) > th["turn_gyro"]:
            self.state = TURNING
            return self.state

        # 10. 前进
        if horiz_accel > th["move_accel"]:
            self.state = MOVING_FORWARD
            return self.state

        # 11. 静止
        self.state = STATIONARY
        return self.state

    def _reset_counters(self):
        self._spin_cnt = 0
        self._prev_accel = -1.0
        self._collision_triggered = False
        self._spike_peak = 0.0
        self._accel_win.clear()


# ═══════════════════════════════════════════════════════════
# 4. 控制器（生成速度指令 + 发送接口）
# ═══════════════════════════════════════════════════════════
# 速度预设（可调）
VELOCITY_PROFILE = {
    STATIONARY:         (0.0,  0.0,  "静止 → 原地保持"),
    MOVING_FORWARD:     (0.5,  0.0,  "前进 → 维持速度"),
    TURNING:            (0.2,  30.0, "转弯 → 减速+转向"),
    FALLEN:             (0.0,  0.0,  "⚠️  跌倒 → 紧急停止"),
    STEEP_SLOPE:        (0.1,  0.0,  "⚠️  陡坡 → 减速慢行"),
    ROLLOVER_WARNING:   (0.0,  0.0,  "⚠️  翻转预警 → 停止稳定"),
    ROLLOVER:           (0.0,  0.0,  "🚨 翻转 → 紧急停止"),
    FREEFALL:           (0.0,  0.0,  "🚨 自由落体 → 紧急停止"),
    SENSOR_FAULT:       (0.0,  0.0,  "🚨 传感器故障 → 紧急停止"),
    SPIN:               (0.0,  0.0,  "🚨 急转 → 紧急停止"),
    VIOLENT_COLLISION:  (0.0,  0.0,  "🚨 碰撞 → 紧急停止"),
    VIBRATION_ABNORMAL: (0.2,  0.0,  "⚠️  异常振动 → 降速"),
    UNKNOWN:            (0.0,  0.0,  "初始化中..."),
}


class RobotController:
    def __init__(self):
        self._last_state = UNKNOWN

    def on_state(self, state):
        """状态变化时自动触发发送，相同状态不重复发送"""
        changed = (state != self._last_state)
        self._last_state = state
        linear, angular, reason = VELOCITY_PROFILE.get(state, (0, 0, "未知"))
        if changed:
            self.send_command(state, linear, angular, reason)
        return linear, angular, reason

    def send_command(self, state, linear, angular, reason):
        """
        ══════════════════════════════════════════════
        ★ 在这里接入你的实际控制接口 ★
        ══════════════════════════════════════════════
        Examples:
          # 串口（ROS std_msgs）
          # ser.write(f"{linear},{angular}\n".encode())

          # ROS2 Twist
          # msg.linear.x  = linear
          # msg.angular.z = math.radians(angular)
          # pub.publish(msg)

          # Socket
          # sock.send(json.dumps({...}).encode())
        """
        pass   # 当前仅显示，不发送硬件指令


# ═══════════════════════════════════════════════════════════
# 5. 主程序：实时显示
# ═══════════════════════════════════════════════════════════
running = True
def _sig(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT, _sig)


def main():
    bias   = load_calibration()
    mpu    = MPU6050Driver(bus=1, bias=bias)
    filt   = ComplementaryFilter(alpha=0.96)
    est    = StateEstimator()
    ctrl   = RobotController()

    # 等待滤波器收敛（约 10 帧）
    print("\n[系统] 初始化中，请保持静止 1 秒...\n")
    for _ in range(10):
        d = mpu.read()
        filt.update(d)
        time.sleep(0.02)

    print("按 Ctrl+C 退出\n")

    prev_state = UNKNOWN
    frame = 0

    while running:
        t0 = time.monotonic()

        d = mpu.read()
        roll, pitch, yaw = filt.update(d)
        state = est.update(d, roll, pitch)
        linear, angular, reason = ctrl.on_state(state)

        # 终端刷新（覆盖前 15 行）
        if frame > 0:
            print("\033[15A", end="")   # 上移 15 行

        color = STATE_COLOR.get(state, "")
        ts    = time.strftime("%H:%M:%S")

        print("╔══════════════════════════════════════════════════════╗")
        print(f"║   Guardian Pi  —  IMU 实时状态监控       {ts}  ║")
        print("╠══════════════════════════════════════════════════════╣")
        print(f"║  [姿态角]                                            ║")
        print(f"║    Roll  = {roll:>8.2f} °                               ║")
        print(f"║    Pitch = {pitch:>8.2f} °                               ║")
        print(f"║    Yaw   = {yaw:>8.2f} ° (积分，仅供参考)            ║")
        print("╠══════════════════════════════════════════════════════╣")
        print(f"║  [机器人状态]  {color}{state:<20}{RESET}                  ║")
        print("╠══════════════════════════════════════════════════════╣")
        print(f"║  [控制指令]                                          ║")
        print(f"║    线速度  = {linear:>5.2f} m/s                           ║")
        print(f"║    角速度  = {angular:>5.1f} °/s                            ║")
        print(f"║    原因    : {reason:<40}║")
        print("╠══════════════════════════════════════════════════════╣")
        print(f"║  [传感器]  az={d['az']:+.3f}g  T={d['temp']:.1f}℃           "
              f"帧#{frame:<6}║")
        print("╚══════════════════════════════════════════════════════╝")

        frame += 1
        dt = time.monotonic() - t0
        sleep = max(0.0, 0.02 - dt)   # 50 Hz
        time.sleep(sleep)

    print("\n\n已退出。")


if __name__ == "__main__":
    main()
