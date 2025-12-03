
# MPU6500 传感器驱动 (ESP-IDF)

[![许可证: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![ESP-IDF 版本](https://img.shields.io/badge/ESP--IDF-5.5.1%2B-blue)
![平台](https://img.shields.io/badge/平台-ESP32%20%7C%20ESP32--S2%20%7C%20ESP32--C3%20%7C%20ESP32--S3-green)

专为 ESP-IDF 框架设计的 MPU6500 六轴 IMU（加速度计 + 陀螺仪）驱动程序。包含传感器校准、实时姿态估计和多数据输出格式。

README_EN [ENGLISH](README_en.md)

## 🚀 快速开始

### 前提条件
- ESP-IDF v5.5.1 或更高版本
- ESP32、ESP32-S2、ESP32-C3 或 ESP32-S3
- MPU6500 或 MPU6500 扩展板
- 基础 I2C 接线（SCL、SDA、VCC、GND）

### 硬件连接
| MPU6500 引脚 | ESP32 引脚 | 说明 |
|-------------|-----------|-------------|
| VCC         | 3.3V      | 3.3V 电源   |
| GND         | GND       | 地线        |
| SCL         | GPIO14    | I2C 时钟线  |
| SDA         | GPIO13    | I2C 数据线  |
| AD0         | GND/3.3V  | I2C 地址选择 (GND=0x68, 3.3V=0x69) |

### 安装
1. 克隆此仓库到您项目的 components 目录：
```bash
cd your-project/components
git clone https://github.com/cengizsinankostakoglu/mpu6500-driver.git
2.在主应用程序中配置 I2C 引脚：

#define I2C_SCL_PIN     14
#define I2C_SDA_PIN     13
#define I2C_FREQ_HZ     400000

📖 功能特性
核心功能
✅ 完整的 MPU6500 寄存器级控制

✅ 自动简单水平加速度计校准

✅ 陀螺仪零偏校准

✅ 姿态估计 Mahony 滤波器

✅ 四元数和欧拉角输出

✅ NVS 存储校准参数


✅ 温度传感器支持

✅ 电源管理（睡眠/唤醒）

✅ 中断驱动数据就绪

高级功能
实时传感器诊断

可配置滤波器参数（KP、KI、beta）

可调采样率（1Hz - 1kHz）

传感器健康监测

自动校准检测

跨平台兼容性

🛠️ 使用示例
基础使用

请参照项目example文件夹中的basic_example.c,将其内容复制并覆盖到你的main.c中,然后编译并运行，通过串口观测输出
如果需要验证高级功能可以使用advaned_example.c，将其内容复制并覆盖到你的main.c中,然后编译并运行，通过串口观测输出

📁 项目结构
mpu6500-driver/
├── include
|   └── mpu6500.h                   # 主头文件
|——example
|    |── basic_example.c            # 基础使用示例
|     └──   advanced_example.c      # 完整功能示例
├── mpu6500.c                       # 驱动实现
├── CMakeLists.txt                  # 构建配置
├── README.md                       # 英文文档
├── README_CN.md                    # 中文文档
└── LICENSE                         # MIT 许可证
🔧 配置
传感器量程配置 (在 mpu6500.h 中)

// 加速度计量程: ±2g, ±4g, ±8g, ±16g
#define ACCEL_FS 0x00       // ±2g

// 陀螺仪量程: ±250, ±500, ±1000, ±2000°/s
#define GYRO_FS (0x01<<3)   // ±500°/s

滤波器配置

mpu_filter_config_t filter_config = {
    .sample_rate_hz = 80.0f,    // 匹配您的系统时钟频率
    .kp = 4.0f,                 // 比例增益
    .ki = 0.01f,                // 积分增益
    .beta = 0.05f               // 陀螺仪零偏估计增益
};

🎮 交互式命令 (高级示例)
运行高级示例并使用以下串口命令：

命令	描述
h	显示帮助菜单
c	开始传感器校准
t	测试传感器功能
j	JSON 输出模式
s	开始连续数据采集
q	退出程序

📊 输出格式
详细模式

[       0] Roll=  0.23°, Pitch= -1.45°, Yaw= 178.92° | Quat: [0.999, 0.002, 0.031, 0.001] | Gyro: [  0.1,   0.2,  -0.3] °/s

JSON 模式

{"ts":123456789,"accel":[0.003,0.015,1.002],"gyro":[0.12,0.23,-0.15],"euler":[0.23,-1.45,178.92]}

🐛 故障排除
常见问题
1. 传感器未检测到

检查 I2C 连接和上拉电阻（推荐 4.7kΩ）

验证 I2C 地址（AD0=GND 时为 0x68，AD0=3.3V 时为 0x69）

确保电源稳定（3.3V）

2. 姿态估计不准

调整 mpu6500.h 中的 SAMPLE_RATE_HZ 以匹配您的系统

调整滤波器参数：增加 kp 加快响应，减少提高稳定性

执行正确的传感器校准

3. 校准时间错误

在 menuconfig 中将 configTICK_RATE_HZ 设为 1000：
idf.py menuconfig > Component config > FreeRTOS > Tick rate (Hz)

4. 数据溢出

如果使用高滤波器增益，降低采样率

检查传感器是否饱和（超过 ±2g 或 ±500°/s）

📈 性能
指标	值	备注
最大采样率	1 kHz	受 I2C 速度限制
默认采样率	80 Hz	适用于大多数应用
滤波器延迟	< 5 ms	Mahony 滤波器计算
功耗	3.9 mA	正常模式，所有传感器激活
校准时间	6 秒	简单水平校准与陀螺仪校准
内存使用	~8 KB	包含滤波器状态

🤝 贡献指南
欢迎贡献！您可以通过以下方式帮助：

报告错误：提交问题并提供详细描述

建议功能：分享您的改进想法

提交代码：fork 仓库并创建拉取请求

改进文档：修正拼写错误或添加示例

开发环境设置
本项目基于vscode+espidf插件的环境开发，请将项目复制到你的组件目录内并在根目录的CMakelists.txt设置好好编译路径
例如：


# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(yourproject)
set(EXTRA_COMPONENT_DIRS ./components/MPU6500)



📄 许可证
本项目采用 MIT 许可证 - 查看 LICENSE 文件了解详情。

🙏 致谢
InvenSense 提供 MPU6500 传感器

Espressif Systems 提供 ESP-IDF 框架

Sebastian Madgwick 提供 Mahony 滤波器算法

所有驱动程序的贡献者和测试者（虽然目前都是我）

📞 支持
如需支持，请：

查看 故障排除 部分

搜索现有 问题

创建新问题时提供：

ESP-IDF 版本

硬件配置

错误日志

重现步骤

⭐ 如果这个项目对您有帮助，请给它一个 star！ ⭐


作者的话：
我为什么要写这个呢？其实是因为晚上两点买imu买错了，然后去espidf组件管理器和github逛了一圈都没有开
箱即用的驱动（虽然找到过一个c++开发的驱动库，但是我目前还不是很清楚C++，理解函数结构也要不少时间），
故而花了两天写了这个简单的驱动库，如果有人有兴趣的话可以协助我开发dmp和其他高级功能。还有一个原因是
价格，tb上mpu6050基本要八九块，mou6500就只需要5.5，从这里也可见产品资料的重要性了