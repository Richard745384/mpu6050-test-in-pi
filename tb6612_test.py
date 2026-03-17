#!/usr/bin/env python3
"""
tb6612_test.py — TB6612 电机驱动测试
====================================
硬件接线：
  PCA9685 (I2C 0x40, GPIO2/3)
    CH0 → TB6612 PWMA  (电机A 速度)
    CH1 → TB6612 PWMB  (电机B 速度)

  树莓派 GPIO → TB6612
    GPIO17 → AIN1  (电机A 方向)
    GPIO27 → AIN2  (电机A 方向)
    GPIO22 → BIN1  (电机B 方向)
    GPIO23 → BIN2  (电机B 方向)
    GPIO24 → STBY  (使能)

  TB6612 电源：
    VCC  → 3.3V
    VM   → 外部电源 (6~12V)
    GND  → 共地

运行：
  python3 tb6612_test.py
"""

import smbus
import time
import signal
import sys

try:
    import RPi.GPIO as GPIO
except ImportError:
    print("错误：需要 RPi.GPIO 库。请在树莓派上运行此脚本。")
    sys.exit(1)


# ═══════════════════════════════════════════════════════════
# 1. PCA9685 PWM 驱动
# ═══════════════════════════════════════════════════════════
class PCA9685:
    """通过 I2C 控制 PCA9685 16 通道 PWM 驱动器"""

    # 寄存器地址
    MODE1       = 0x00
    PRESCALE    = 0xFE
    LED0_ON_L   = 0x06
    LED0_ON_H   = 0x07
    LED0_OFF_L  = 0x08
    LED0_OFF_H  = 0x09

    def __init__(self, bus=1, addr=0x40):
        self.bus  = smbus.SMBus(bus)
        self.addr = addr
        self._init()

    def _init(self):
        """初始化 PCA9685"""
        self.bus.write_byte_data(self.addr, self.MODE1, 0x00)  # 正常模式
        time.sleep(0.005)
        self.set_pwm_freq(1000)  # 1kHz PWM 频率，适合有刷直流电机
        print(f"[PCA9685] 初始化完成，地址 0x{self.addr:02X}，PWM 频率 1000Hz")

    def set_pwm_freq(self, freq_hz):
        """设置 PWM 频率 (Hz)"""
        # PCA9685 内部时钟 25MHz
        prescale = int(round(25000000.0 / (4096 * freq_hz)) - 1)
        prescale = max(3, min(255, prescale))

        old_mode = self.bus.read_byte_data(self.addr, self.MODE1)
        # 进入 SLEEP 模式才能修改 PRESCALE
        self.bus.write_byte_data(self.addr, self.MODE1, (old_mode & 0x7F) | 0x10)
        self.bus.write_byte_data(self.addr, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.addr, self.MODE1, old_mode)
        time.sleep(0.005)
        # 开启自动递增
        self.bus.write_byte_data(self.addr, self.MODE1, old_mode | 0xA0)

    def set_pwm(self, channel, on, off):
        """
        设置指定通道的 PWM 值
        channel: 0~15
        on:  上升沿计数 (0~4095)
        off: 下降沿计数 (0~4095)
        """
        base = self.LED0_ON_L + 4 * channel
        self.bus.write_byte_data(self.addr, base,     on & 0xFF)
        self.bus.write_byte_data(self.addr, base + 1, on >> 8)
        self.bus.write_byte_data(self.addr, base + 2, off & 0xFF)
        self.bus.write_byte_data(self.addr, base + 3, off >> 8)

    def set_duty(self, channel, duty):
        """
        设置占空比
        duty: 0.0 ~ 1.0
        """
        duty = max(0.0, min(1.0, duty))
        if duty == 0:
            self.set_pwm(channel, 0, 0)
        elif duty >= 1.0:
            self.set_pwm(channel, 4096, 0)  # 全高 (特殊位)
        else:
            off_val = int(duty * 4095)
            self.set_pwm(channel, 0, off_val)

    def stop_all(self):
        """关闭所有通道"""
        for ch in range(16):
            self.set_pwm(ch, 0, 0)


# ═══════════════════════════════════════════════════════════
# 2. TB6612 电机控制
# ═══════════════════════════════════════════════════════════

# GPIO 引脚定义 (BCM 编号)
PIN_AIN1 = 17   # 电机A 方向1
PIN_AIN2 = 27   # 电机A 方向2
PIN_BIN1 = 22   # 电机B 方向1
PIN_BIN2 = 23   # 电机B 方向2
PIN_STBY = 24   # TB6612 使能

# PCA9685 通道定义
CH_PWMA = 0     # 电机A 速度
CH_PWMB = 1     # 电机B 速度


class TB6612:
    """TB6612FNG 双电机驱动控制"""

    def __init__(self, pwm_driver: PCA9685):
        self.pwm = pwm_driver
        self._setup_gpio()

    def _setup_gpio(self):
        """初始化 GPIO 引脚"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in [PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2, PIN_STBY]:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        # 使能 TB6612
        GPIO.output(PIN_STBY, GPIO.HIGH)
        print("[TB6612] GPIO 初始化完成，STBY=HIGH (已使能)")

    def motor_a(self, speed):
        """
        控制电机A
        speed: -1.0 ~ 1.0  (正=正转, 负=反转, 0=停止)
        """
        self._drive(PIN_AIN1, PIN_AIN2, CH_PWMA, speed)

    def motor_b(self, speed):
        """
        控制电机B
        speed: -1.0 ~ 1.0  (正=正转, 负=反转, 0=停止)
        """
        self._drive(PIN_BIN1, PIN_BIN2, CH_PWMB, speed)

    def _drive(self, pin1, pin2, pwm_ch, speed):
        """驱动单个电机"""
        speed = max(-1.0, min(1.0, speed))

        if speed > 0:
            # 正转
            GPIO.output(pin1, GPIO.HIGH)
            GPIO.output(pin2, GPIO.LOW)
            self.pwm.set_duty(pwm_ch, speed)
        elif speed < 0:
            # 反转
            GPIO.output(pin1, GPIO.LOW)
            GPIO.output(pin2, GPIO.HIGH)
            self.pwm.set_duty(pwm_ch, -speed)
        else:
            # 停止 (滑行)
            GPIO.output(pin1, GPIO.LOW)
            GPIO.output(pin2, GPIO.LOW)
            self.pwm.set_duty(pwm_ch, 0)

    def brake_a(self):
        """电机A 刹车 (短路制动)"""
        GPIO.output(PIN_AIN1, GPIO.HIGH)
        GPIO.output(PIN_AIN2, GPIO.HIGH)
        self.pwm.set_duty(CH_PWMA, 0)
        print("  电机A 刹车")

    def brake_b(self):
        """电机B 刹车 (短路制动)"""
        GPIO.output(PIN_BIN1, GPIO.HIGH)
        GPIO.output(PIN_BIN2, GPIO.HIGH)
        self.pwm.set_duty(CH_PWMB, 0)
        print("  电机B 刹车")

    def stop(self):
        """停止所有电机 (滑行停止)"""
        self.motor_a(0)
        self.motor_b(0)
        print("  所有电机已停止")

    def brake_all(self):
        """所有电机刹车"""
        self.brake_a()
        self.brake_b()

    def standby(self, enable=True):
        """设置待机/使能"""
        GPIO.output(PIN_STBY, GPIO.HIGH if enable else GPIO.LOW)
        print(f"  STBY = {'HIGH (使能)' if enable else 'LOW (待机)'}")

    def cleanup(self):
        """清理资源"""
        self.stop()
        GPIO.output(PIN_STBY, GPIO.LOW)
        self.pwm.stop_all()
        GPIO.cleanup()
        print("[TB6612] 已清理 GPIO 和 PWM")


# ═══════════════════════════════════════════════════════════
# 3. 测试函数
# ═══════════════════════════════════════════════════════════

def test_single_motor(tb, motor_name, motor_func):
    """测试单个电机：正转 → 反转 → 停止"""
    print(f"\n--- 测试{motor_name} ---")

    print(f"  {motor_name} 正转 50% 速度...")
    motor_func(0.5)
    time.sleep(2)

    print(f"  {motor_name} 正转 100% 速度...")
    motor_func(1.0)
    time.sleep(2)

    print(f"  {motor_name} 反转 50% 速度...")
    motor_func(-0.5)
    time.sleep(2)

    print(f"  {motor_name} 反转 100% 速度...")
    motor_func(-1.0)
    time.sleep(2)

    print(f"  {motor_name} 停止")
    motor_func(0)
    time.sleep(1)


def test_both_motors(tb):
    """同时驱动两个电机"""
    print("\n--- 双电机同步测试 ---")

    print("  双电机正转 60%...")
    tb.motor_a(0.6)
    tb.motor_b(0.6)
    time.sleep(2)

    print("  双电机反转 60%...")
    tb.motor_a(-0.6)
    tb.motor_b(-0.6)
    time.sleep(2)

    print("  差速转弯 (A=80%, B=30%)...")
    tb.motor_a(0.8)
    tb.motor_b(0.3)
    time.sleep(2)

    print("  原地旋转 (A=正转, B=反转 50%)...")
    tb.motor_a(0.5)
    tb.motor_b(-0.5)
    time.sleep(2)

    tb.stop()
    time.sleep(1)


def test_ramp(tb):
    """渐变加速/减速测试"""
    print("\n--- 渐变加速/减速测试 ---")
    steps = 20
    delay = 0.15

    print("  加速中...")
    for i in range(steps + 1):
        speed = i / steps
        tb.motor_a(speed)
        tb.motor_b(speed)
        bar = "█" * (i * 2) + "░" * ((steps - i) * 2)
        print(f"\r  速度: {speed:5.1%}  [{bar}]", end="", flush=True)
        time.sleep(delay)
    print()

    print("  减速中...")
    for i in range(steps, -1, -1):
        speed = i / steps
        tb.motor_a(speed)
        tb.motor_b(speed)
        bar = "█" * (i * 2) + "░" * ((steps - i) * 2)
        print(f"\r  速度: {speed:5.1%}  [{bar}]", end="", flush=True)
        time.sleep(delay)
    print()

    tb.stop()
    time.sleep(1)


def test_brake(tb):
    """刹车测试"""
    print("\n--- 刹车测试 ---")
    print("  双电机正转 80%...")
    tb.motor_a(0.8)
    tb.motor_b(0.8)
    time.sleep(2)

    print("  刹车!")
    tb.brake_all()
    time.sleep(2)


def test_custom(tb):
    """自定义速度控制"""
    print("\n--- 自定义速度控制 ---")
    print("  输入速度值 (-1.0 ~ 1.0)，正=正转，负=反转")
    print("  输入 'q' 返回主菜单\n")

    while True:
        try:
            inp = input("  电机A 速度 (或 'q' 退出): ").strip()
            if inp.lower() == 'q':
                tb.stop()
                break
            speed_a = float(inp)

            inp = input("  电机B 速度 (或 'q' 退出): ").strip()
            if inp.lower() == 'q':
                tb.stop()
                break
            speed_b = float(inp)

            tb.motor_a(speed_a)
            tb.motor_b(speed_b)
            print(f"  → 电机A={speed_a:+.2f}  电机B={speed_b:+.2f}\n")

        except ValueError:
            print("  ⚠ 请输入有效数字\n")


# ═══════════════════════════════════════════════════════════
# 4. 主程序
# ═══════════════════════════════════════════════════════════
BANNER = """
╔══════════════════════════════════════════════════╗
║       TB6612 电机驱动测试 — 树莓派 5             ║
╠══════════════════════════════════════════════════╣
║  PCA9685 (I2C 0x40)  →  TB6612 PWMA/PWMB       ║
║  GPIO17/27           →  TB6612 AIN1/AIN2        ║
║  GPIO22/23           →  TB6612 BIN1/BIN2        ║
║  GPIO24              →  TB6612 STBY             ║
╚══════════════════════════════════════════════════╝
"""

MENU = """
  ┌─────────────────────────────────┐
  │  1. 测试电机A (正转/反转/停止)  │
  │  2. 测试电机B (正转/反转/停止)  │
  │  3. 双电机同步测试              │
  │  4. 渐变加速/减速测试           │
  │  5. 刹车测试                    │
  │  6. 自定义速度控制              │
  │  7. 紧急停止                    │
  │  0. 退出                        │
  └─────────────────────────────────┘
"""


def main():
    print(BANNER)

    # 注册信号处理 - 确保 Ctrl+C 能正常清理
    tb = None

    def cleanup_handler(sig, frame):
        print("\n\n[信号] 收到中断信号，正在清理...")
        if tb:
            tb.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup_handler)
    signal.signal(signal.SIGTERM, cleanup_handler)

    try:
        # 初始化硬件
        print("[初始化] 正在连接 PCA9685...")
        pwm = PCA9685(bus=1, addr=0x40)

        print("[初始化] 正在初始化 TB6612...")
        tb = TB6612(pwm)

        print("\n✅ 硬件初始化完成！\n")

        # 交互式菜单
        while True:
            print(MENU)
            choice = input("  请选择 [0-7]: ").strip()

            if choice == '1':
                test_single_motor(tb, "电机A", tb.motor_a)
            elif choice == '2':
                test_single_motor(tb, "电机B", tb.motor_b)
            elif choice == '3':
                test_both_motors(tb)
            elif choice == '4':
                test_ramp(tb)
            elif choice == '5':
                test_brake(tb)
            elif choice == '6':
                test_custom(tb)
            elif choice == '7':
                print("\n  🛑 紧急停止!")
                tb.brake_all()
            elif choice == '0':
                print("\n退出测试...")
                break
            else:
                print("  ⚠ 无效选项，请重新选择")

    except OSError as e:
        print(f"\n❌ 硬件连接错误: {e}")
        print("   请检查：")
        print("   1. PCA9685 是否正确连接到 I2C (GPIO2/3)")
        print("   2. 运行 'sudo i2cdetect -y 1' 检查 I2C 设备")
        print("   3. 是否已启用 I2C: sudo raspi-config → Interface → I2C")

    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()

    finally:
        if tb:
            tb.cleanup()
        print("再见！")


if __name__ == "__main__":
    main()
