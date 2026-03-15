# MPU6050 & Camera Test in Raspberry Pi

这是一个用于树莓派的 MPU6050 传感器和相机模块的测试代码库。它不仅包含了基础的数据读取和校准，还提供了一个通过互补滤波器（Complementary Filter）进行姿态解算和状态识别（如前进、跌倒、急转等）的完整演示。此外，也包含了一个自动适配树莓派各代相机及 USB 摄像头的拍照脚本。

## 硬件连接说明

**MPU6050 到树莓派的引脚连接：**
* **VCC** -> 3.3V (Pin 1)
* **GND** -> GND (Pin 6)
* **SCL** -> SCL (Pin 5 / GPIO 3)
* **SDA** -> SDA (Pin 3 / GPIO 2)
* **AD0** -> GND (使 I2C 地址为 `0x68`)

## 依赖安装

在使用本项目之前，请确保你已经安装了以下依赖，并在树莓派配置中开启了 I2C：

```bash
# 1. 开启 I2C 接口
sudo raspi-config
# 选择 Interface Options -> I2C -> Enable -> Yes

# 2. 安装 Python 依赖和 i2c-tools
sudo apt-get update
sudo apt-get install -y python3-smbus i2c-tools
```

如果你需要运行 `take_photo.py`（拍照脚本），请确保安装了相机相关的依赖：
```bash
sudo apt-get install -y python3-opencv python3-picamera
# 如果你使用的是较新的树莓派系统（Bullseye/Bookworm）和官方 libcamera
sudo apt-get install -y libcamera-apps
```

## 文件及使用说明

### 1. 传感器校准
文件：`mpu6050_calibrate.py`

**使用方法：**
将 MPU6050 模块**完全水平且静止**放置在桌面上，运行以下命令：
```bash
python3 mpu6050_calibrate.py
```
程序会自动采集约 5 秒的数据，计算出传感器的零偏（Bias），并自动在当前目录下生成配置文件 `calibration.json`。**在运行其他读取或控制脚本前，一定要先完成这一步。**

### 2. 读取基础原始数据与姿态估算
文件：`mpu6050_reader.py`

**使用方法：**
```bash
python3 mpu6050_reader.py
```
这会持续打印 MPU6050 的加速度、角速度、温度以及一个初步的静态姿态角（只用加速度计计算的低动态 Roll / Pitch）。按 `Ctrl+C` 退出。

### 3. 机器人状态监控与控制生成（高级）
文件：`robot_controller.py`

**使用方法：**
```bash
python3 robot_controller.py
```
这个脚本极其强大，它模拟了机器人的 IMU 解算引擎：
* 利用**互补滤波器 (Complementary Filter)** 将加速度计和陀螺仪的数据进行融合，得到准确、平滑的动态 Roll, Pitch, Yaw（航向角）。
* 内置**状态机算法**，实时分析传感器的跳变、旋转加速度等，识别并显示出 12 种机器人状态，例如：**移动中**、**急转**、**自由落体 (Freefall)**、**陡坡 (Steep Slope)**、**跌倒 (Fallen)** 和**剧烈碰撞 (Violent Collision)** 等。
* 自动生成对应的底层线速度/角速度**控制指令建议**。如果要在你的机器人上使用，只需在脚本底部的 `send_command` 函数内将指令通过串口、ROS 或 Socket 发送出去即可。

### 4. 万能拍照脚本
文件：`take_photo.py`

**使用方法：**
```bash
python3 take_photo.py
# 或指定文件名
python3 take_photo.py my_awesome_photo.jpg
```
一个智能适配脚本，能自动检测你的树莓派环境并决定使用 `picamera2`、旧版本的 `picamera` 还是基于 OpenCV 的 USB 摄像头。运行一次即保存一张照片，兼容性极高。

## 许可证
MIT License
