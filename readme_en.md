# MPU6500 Sensor Driver (ESP-IDF)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
![ESP-IDF Version](https://img.shields.io/badge/ESP--IDF-5.5.1%2B-blue)
![Platform](https://img.shields.io/badge/Platform-ESP32%20%7C%20ESP32--S2%20%7C%20ESP32--C3%20%7C%20ESP32--S3-green)

A driver for the MPU6500 6-axis IMU (accelerometer + gyroscope) designed for the ESP-IDF framework. Includes sensor calibration, real-time attitude estimation, and multiple data output formats.
zh_CN [简体中文](README_CN.md)
## 🚀 Quick Start

### Prerequisites
- ESP-IDF v5.5.1 or later
- ESP32, ESP32-S2, ESP32-C3, or ESP32-S3
- MPU6500 or MPU6500 expansion board
- Basic I2C wiring (SCL, SDA, VCC, GND)

### Hardware Connection
| MPU6500 Pin | ESP32 Pin | Description |
|-------------|-----------|-------------|
| VCC         | 3.3V      | 3.3V Power  |
| GND         | GND       | Ground      |
| SCL         | GPIO14    | I2C Clock   |
| SDA         | GPIO13    | I2C Data    |
| AD0         | GND/3.3V  | I2C Address Select (GND=0x68, 3.3V=0x69) |

### Installation
1. Clone this repository to your project's `components` directory:
```bash
cd your-project/components
git clone https://github.com/cengizsinankostakoglu/mpu6500-driver.git
Configure I2C pins in your main application:

c
#define I2C_SCL_PIN     14
#define I2C_SDA_PIN     13
#define I2C_FREQ_HZ     400000

📖 Features
Core Features
✅ Complete MPU6500 register-level control

✅ Automatic simple level accelerometer calibration

✅ Gyroscope bias calibration

✅ Mahony filter for attitude estimation

✅ Quaternion and Euler angle output

✅ NVS storage for calibration parameters

✅ Temperature sensor support

✅ Power management (sleep/wake)

✅ Interrupt-driven data ready

Advanced Features
Real-time sensor diagnostics

Configurable filter parameters (KP, KI, beta)

Adjustable sample rate (1Hz - 1kHz)

Sensor health monitoring

Automatic calibration detection

Cross-platform compatibility

🛠️ Usage Examples
Basic Usage
Refer to basic_example.c in the project's example folder. Copy its content into your main.c, then compile and run. Observe the output via serial port.

Advanced Features
To verify advanced features, use advanced_example.c. Copy its content into your main.c, then compile and run. Observe the output via serial port.

📁 Project Structure
text
mpu6500-driver/
├── include
│   └── mpu6500.h                   # Main header file
├── example
│   ├── basic_example.c            # Basic usage example
│   └── advanced_example.c         # Full-feature example
├── mpu6500.c                      # Driver implementation
├── CMakeLists.txt                 # Build configuration
├── README.md                      # English documentation
├── README_CN.md                   # Chinese documentation
└── LICENSE                        # MIT License
🔧 Configuration
Sensor Range Configuration (in mpu6500.h)
c
// Accelerometer range: ±2g, ±4g, ±8g, ±16g
#define ACCEL_FS 0x00       // ±2g

// Gyroscope range: ±250, ±500, ±1000, ±2000 °/s
#define GYRO_FS (0x01<<3)   // ±500 °/s
Filter Configuration
c
mpu_filter_config_t filter_config = {
    .sample_rate_hz = 80.0f,    // Match your system clock frequency
    .kp = 4.0f,                 // Proportional gain
    .ki = 0.01f,                // Integral gain
    .beta = 0.05f               // Gyroscope bias estimation gain
};
🎮 Interactive Commands (Advanced Example)
Run the advanced example and use the following serial commands:

Command	Description
h	Show help menu
c	Start sensor calibration
t	Test sensor functionality
j	JSON output mode
s	Start continuous data acquisition
q	Quit program
📊 Output Formats
Verbose Mode
text
[       0] Roll=  0.23°, Pitch= -1.45°, Yaw= 178.92° | Quat: [0.999, 0.002, 0.031, 0.001] | Gyro: [  0.1,   0.2,  -0.3] °/s
JSON Mode
json
{"ts":123456789,"accel":[0.003,0.015,1.002],"gyro":[0.12,0.23,-0.15],"euler":[0.23,-1.45,178.92]}
🐛 Troubleshooting
Common Issues
Sensor not detected

Check I2C connections and pull-up resistors (recommended 4.7kΩ)

Verify I2C address (0x68 when AD0=GND, 0x69 when AD0=3.3V)

Ensure stable power supply (3.3V)

Inaccurate attitude estimation

Adjust SAMPLE_RATE_HZ in mpu6500.h to match your system

Tune filter parameters: increase kp for faster response, decrease for stability

Perform proper sensor calibration

Calibration timing error

Set configTICK_RATE_HZ to 1000 in menuconfig:
idf.py menuconfig > Component config > FreeRTOS > Tick rate (Hz)

Data overflow

Reduce sample rate if using high filter gains

Check if sensor is saturated (exceeding ±2g or ±500 °/s)

📈 Performance
Metric	Value	Notes
Maximum Sample Rate	1 kHz	Limited by I2C speed
Default Sample Rate	80 Hz	Suitable for most applications
Filter Latency	< 5 ms	Mahony filter computation
Power Consumption	3.9 mA	Normal mode, all sensors active
Calibration Time	6 sec	Simple level calibration & gyro calibration
Memory Usage	~8 KB	Includes filter state
🤝 Contributing
Contributions are welcome! You can help by:

Reporting bugs: Submit issues with detailed descriptions

Suggesting features: Share your improvement ideas

Submitting code: Fork the repository and create a pull request

Improving documentation: Fix typos or add examples

Development Environment Setup
This project is developed in a VSCode + ESP-IDF plugin environment. Copy the project to your components directory and set the compilation path correctly in the root directory's CMakeLists.txt.

Example:

cmake
# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(yourproject)
set(EXTRA_COMPONENT_DIRS ./components/MPU6500)
📄 License
This project is licensed under the MIT License - see the LICENSE file for details.

🙏 Acknowledgments
InvenSense for the MPU6500 sensor

Espressif Systems for the ESP-IDF framework

Sebastian Madgwick for the Mahony filter algorithm

All contributors and testers of the driver (though currently it's just me)

📞 Support
For support, please:

Check the Troubleshooting section

Search existing Issues

When creating a new issue, provide:

ESP-IDF version

Hardware configuration

Error logs

Steps to reproduce

⭐ If this project helps you, please give it a star! ⭐