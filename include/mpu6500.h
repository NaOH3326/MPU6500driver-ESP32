/**
 * @file mpu6500.h 
 * @brief MPU6500 accelerometer and gyroscope driver for ESP-IDF
 * @details This file contains the interface for the
 *          MPU6500 accelerometer and gyroscope driver, supporting various
 *          operations.
 * @author Cengiz Sinan Kostakoglu
 * @version 1.0
 * @date 2025-06-08
 */

#ifndef MPU6500_H
#define MPU6500_H

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include "sdkconfig.h"


#ifdef __cplusplus
extern "C" {
#endif

#define GYRO_FS (0x01<<3)   // ±500°/s,可自行修改
#define ACCEL_FS 0x00       // ±2g ,可自行修改
#define GYRO_SENSITIVITY 65.5f // LSB/°/s for ±500°/s,可自行修改
#define ACCEL_SENSITIVITY 16384.0f // LSB/g for ±2g，可自行修改

//如果出现角度不准请修改此项，非常重要！！！滤波器工作需要一个精确的dt，而每个人的系统频率不一样
#define SAMPLE_RATE_HZ  80.0f 



/* MPU6500 I2C Address */
#define MPU6500_ADDR        0x68 // AD0 = 0 -> 0x68 || AD0 = 1 -> 0x69

/* MPU6500 Register Addresses */
#define SELF_TEST_X_GYRO    0x00
#define SELF_TEST_Y_GYRO    0x01
#define SELF_TEST_Z_GYRO    0x02
#define SELF_TEST_X_ACCEL   0x0D
#define SELF_TEST_Y_ACCEL   0x0E
#define SELF_TEST_Z_ACCEL   0x0F
#define XG_OFFSET_H         0x13
#define XG_OFFSET_L         0x14
#define YG_OFFSET_H         0x15
#define YG_OFFSET_L         0x16
#define ZG_OFFSET_H         0x17
#define ZG_OFFSET_L         0x18
#define SMPLRT_DIV          0x19
#define CONFIG              0x1A
#define GYRO_CONFIG         0x1B
#define ACCEL_CONFIG        0x1C
#define ACCEL_CONFIG_2      0x1D
#define LP_ACCEL_ODR        0x1E
#define WOM_THR             0x1F
#define FIFO_EN             0x23
#define I2C_MST_CTRL        0x24
#define I2C_SLV0_ADDR       0x25
#define I2C_SLV0_REG        0x26
#define I2C_SLV0_CTRL       0x27
#define I2C_SLV1_ADDR       0x28
#define I2C_SLV1_REG        0x29
#define I2C_SLV1_CTRL       0x2A
#define I2C_SLV2_ADDR       0x2B
#define I2C_SLV2_REG        0x2C
#define I2C_SLV2_CTRL       0x2D
#define I2C_SLV3_ADDR       0x2E
#define I2C_SLV3_REG        0x2F
#define I2C_SLV3_CTRL       0x30
#define I2C_SLV4_ADDR       0x31
#define I2C_SLV4_REG        0x32
#define I2C_SLV4_DO         0x33
#define I2C_SLV4_CTRL       0x34
#define I2C_SLV4_DI         0x35
#define I2C_MST_STATUS      0x36
#define INT_PIN_CFG         0x37
#define INT_ENABLE          0x38
#define INT_STATUS          0x3A
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x4E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_XOUT_L         0x44
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define EXT_SENS_DATA_00    0x49
#define EXT_SENS_DATA_01    0x4A
#define EXT_SENS_DATA_02    0x4B
#define EXT_SENS_DATA_03    0x4C
#define EXT_SENS_DATA_04    0x4D
#define EXT_SENS_DATA_05    0x4E
#define EXT_SENS_DATA_06    0x4F
#define EXT_SENS_DATA_07    0x50
#define EXT_SENS_DATA_08    0x51
#define EXT_SENS_DATA_09    0x52
#define EXT_SENS_DATA_10    0x53
#define EXT_SENS_DATA_11    0x54
#define EXT_SENS_DATA_12    0x55
#define EXT_SENS_DATA_13    0x56
#define EXT_SENS_DATA_14    0x57
#define EXT_SENS_DATA_15    0x58
#define EXT_SENS_DATA_16    0x59
#define EXT_SENS_DATA_17    0x5A
#define EXT_SENS_DATA_18    0x5B
#define EXT_SENS_DATA_19    0x5C
#define EXT_SENS_DATA_20    0x5D
#define EXT_SENS_DATA_21    0x5E
#define EXT_SENS_DATA_22    0x5F
#define EXT_SENS_DATA_23    0x60
#define I2C_SLV0_DO         0x63
#define I2C_SLV1_DO         0x64
#define I2C_SLV2_DO         0x65
#define I2C_SLV3_DO         0x66
#define I2C_MST_DELAY_CTRL  0x67
#define SIGNAL_PATH_RESET   0x68
#define ACCEL_INTEL_CTRL    0x69
#define USER_CTRL           0x6A
#define PWR_MGMT_1          0x6B
#define PWR_MGMT_2          0x6C
#define FIFO_COUNT_H        0x72
#define FIFO_COUNT_L        0x73
#define FIFO_R_W            0x74
#define WHO_AM_I            0x75
#define XA_OFFSET_H         0x77
#define XA_OFFSET_L         0x78
#define YA_OFFSET_H         0x7A
#define YA_OFFSET_L         0x7B
#define ZA_OFFSET_H         0x7D
#define ZA_OFFSET_L         0x7E

/**
 * @brief 姿态数据结构体
 */
typedef struct {
    // 四元数表示
    float q0, q1, q2, q3;
    
    // 欧拉角表示 (度)
    float roll, pitch, yaw;
    
    // 角速度 (度/秒)
    float gyro_x, gyro_y, gyro_z;
    
    // 时间戳和置信度
    uint32_t timestamp_us;
    float confidence;
} attitude_data_t;

/**
 * @brief 加速度计校准数据结构体
 */
typedef struct {
    float accel_bias[3];      // 零偏 (g)
    float accel_scale[3];     // 标度因子
    bool accel_calibrated;
    float gyro_bias[3];
    bool gyro_calibrated;
} calibration_data_t;

/**
 * @brief 滤波器配置结构体
 */
typedef struct {
    float sample_rate_hz;       // 采样频率 (Hz)
    float kp;                   // 比例增益
    float ki;                   // 积分增益
    float beta;                 // 陀螺仪偏差估计增益
} mpu_filter_config_t;

/**
 * @brief MPU6500设备结构体
 */
typedef struct {
    i2c_master_dev_handle_t dev_handle; // I2C设备句柄
    uint8_t i2c_addr;                   // I2C设备地址
} mpu6500_t;

/**
 * @brief 传感器数据读取配置
 */
typedef struct {
    float accel_scale;          // 加速度计量程比例因子
    float gyro_scale;           // 陀螺仪量程比例因子
    bool use_calibration;       // 是否使用校准数据
} sensor_read_config_t;

/**
 * @brief 输出模式枚举
 */
typedef enum {
    OUTPUT_MODE_DETAILED = 0,  // 详细文本输出
    OUTPUT_MODE_SIMPLE,        // 简化输出（适合绘图）
    OUTPUT_MODE_JSON,          // JSON格式输出
    OUTPUT_MODE_QUATERNION,    // 四元数输出
} output_mode_t;

/**
 * @brief 应用程序配置结构体
 */
typedef struct {
    uint8_t scl_pin;            // SCL引脚
    uint8_t sda_pin;            // SDA引脚
    uint32_t i2c_freq_hz;       // I2C频率
    float sample_rate_hz;       // 采样率
    mpu_filter_config_t filter_config;  // 滤波器配置
    output_mode_t output_mode;  // 输出模式
} app_config_t;

/* ======================== MPU6500 默认配置变量 ======================== */

/* ======================== MPU6500 基本驱动函数 ======================== */

/**
 * @brief 初始化MPU6500设备
 * @param bus_handle I2C总线句柄
 * @param mpu MPU6500设备结构体指针
 * @param i2c_addr I2C设备地址
 * @return esp_err_t ESP_OK表示成功，其他表示错误
 */
esp_err_t mpu6500_dev_init(i2c_master_bus_handle_t bus_handle, mpu6500_t* mpu, uint8_t i2c_addr);

/**
 * @brief 初始化MPU6500传感器（配置寄存器）
 * @param mpu MPU6500设备结构体指针
 * @return esp_err_t ESP_OK表示成功，其他表示错误
 */
esp_err_t mpu6500_sensor_init(mpu6500_t* mpu);

/**
 * @brief 读取WHO_AM_I寄存器值
 * @param mpu MPU6500设备结构体指针
 * @param whoami 存储WHO_AM_I值的指针
 * @return esp_err_t ESP_OK表示成功，其他表示错误
 */
esp_err_t mpu6500_read_whoami(mpu6500_t* mpu, uint8_t* whoami);

/**
 * @brief 读取加速度计原始数据
 * @param mpu MPU6500设备结构体指针
 * @param x 存储X轴加速度的指针
 * @param y 存储Y轴加速度的指针
 * @param z 存储Z轴加速度的指针
 * @return esp_err_t ESP_OK表示成功，其他表示错误
 */
esp_err_t mpu6500_read_accel(mpu6500_t* mpu, int16_t* x, int16_t* y, int16_t* z);

/**
 * @brief 读取陀螺仪原始数据
 * @param mpu MPU6500设备结构体指针
 * @param x 存储X轴角速度的指针
 * @param y 存储Y轴角速度的指针
 * @param z 存储Z轴角速度的指针
 * @return esp_err_t ESP_OK表示成功，其他表示错误
 */
esp_err_t mpu6500_read_gyro(mpu6500_t* mpu, int16_t* x, int16_t* y, int16_t* z);

/**
 * @brief 读取温度传感器原始数据
 * @param mpu MPU6500设备结构体指针
 * @param temp 存储温度值的指针
 * @return esp_err_t ESP_OK表示成功，其他表示错误
 */
esp_err_t mpu6500_read_temp(mpu6500_t* mpu, int16_t* temp);

/**
 * @brief 读取传感器数据并转换为物理单位
 * @param mpu MPU6500设备结构体指针
 * @param config 数据读取配置,传入NULL以使用默认值
 * @param accel_g 存储加速度值的数组(g)
 * @param gyro_dps 存储角速度值的数组(°/s)
 * @return esp_err_t ESP_OK表示成功，其他表示错误
 */
esp_err_t mpu6500_read_sensor_data(mpu6500_t* mpu, const sensor_read_config_t* config, 
                                    float accel_g[3], float gyro_dps[3]);

/**
 * @brief 使能数据就绪中断
 * @param mpu MPU6500设备结构体指针
 * @return esp_err_t ESP_OK表示成功，其他表示错误
 */
esp_err_t mpu6500_enable_data_ready_interrupts(mpu6500_t* mpu);

/**
 * @brief 禁用数据就绪中断
 * @param mpu MPU6500设备结构体指针
 * @return esp_err_t ESP_OK表示成功，其他表示错误
 */
esp_err_t mpu6500_disable_data_ready_interrupts(mpu6500_t* mpu);

/**
 * @brief 进入睡眠模式
 * @param mpu MPU6500设备结构体指针
 * @return esp_err_t ESP_OK表示成功，其他表示错误
 */
esp_err_t mpu6500_sleep(mpu6500_t* mpu);

/**
 * @brief 唤醒设备
 * @param mpu MPU6500设备结构体指针
 * @return esp_err_t ESP_OK表示成功，其他表示错误
 */
esp_err_t mpu6500_wakeup(mpu6500_t* mpu);

/**
 * @brief 反初始化MPU6500设备
 * @param mpu MPU6500设备结构体指针
 * @return esp_err_t ESP_OK表示成功，其他表示错误
 */
esp_err_t mpu6500_deinit(mpu6500_t* mpu);

/* ======================== 姿态滤波器函数 ======================== */

/**
 * @brief 初始化姿态滤波器
 * @param config 滤波器配置，为NULL时使用默认配置
 */
void attitude_filter_init(const mpu_filter_config_t* config);

/**
 * @brief 更新姿态滤波器
 * @param accel 加速度计数据 [g]
 * @param gyro 陀螺仪数据 [度/秒]
 * @param dt 时间间隔 [秒]
 * @param output 输出姿态数据
 */
void attitude_filter_update(const float accel[3], const float gyro[3], 
                           float dt, attitude_data_t* output);

/**
 * @brief 重置滤波器状态
 */
void attitude_filter_reset(void);

/**
 * @brief 检查滤波器是否已完成校准
 * @return true 已校准，false 未校准
 */
bool attitude_filter_is_calibrated(void);

/* ======================== NVS存储管理函数 ======================== */

/**
 * @brief 初始化NVS存储系统
 * @return esp_err_t ESP_OK表示成功，其他表示错误
 */
esp_err_t attitude_filter_nvs_init(void);

/**
 * @brief 清除NVS中的校准数据
 * @return esp_err_t ESP_OK表示成功，其他表示错误
 */
esp_err_t attitude_filter_clear_nvs_calibration(void);

/* ======================== 校准函数 ======================== */

/**
 * @brief 保存加速度校准
 * @return void
 */
esp_err_t mpu6500_save_accel_calibration(void);

/**
 * @brief 保存陀螺仪校准
 * @return void
 */
esp_err_t mpu6500_save_gyro_calibration(void);

/**
 * @brief 加载加速度校准
 * @return void
 */
esp_err_t mpu6500_load_accel_calibration(void);

/**
 * @brief 加载陀螺仪校准
 * @return void
 */
esp_err_t mpu6500_load_gyro_calibration(void);

/**
 * @brief 执行简化的加速度计校准（水平放置）
 * @param mpu MPU6500设备结构体指针
 * @param config 传感器读取配置
 * @return esp_err_t ESP_OK表示成功，其他表示错误
 */
esp_err_t mpu6500_calibrate_accel(mpu6500_t* mpu, const sensor_read_config_t* config);

/**
 * @brief 执行简化的陀螺仪零偏校准
 * @param mpu MPU6500设备结构体指针
 * @param config 传感器读取配置
 * @return esp_err_t ESP_OK表示成功，其他表示错误
 */
esp_err_t mpu6500_calibrate_gyro(mpu6500_t* mpu, const sensor_read_config_t* config);

/**
 * @brief 设置校准参数
 * @param gyro_bias 陀螺仪零偏数组[3]
 * @param accel_bias 加速度计校准数组[4] (前3个是零偏，第4个是标度因子)
 */
esp_err_t mpu6500_set_calibration(const float gyro_bias[3], const float accel_bias[6]);

/**
 * @brief 获取校准参数
 * @param gyro_bias 输出陀螺仪零偏数组[3]
 * @param accel_bias 输出加速度计校准数组[4] (前3个是零偏，第4个是标度因子)
 * @return true 获取成功，false 获取失败（未校准）
 */
esp_err_t mpu6500_get_calibration(float gyro_bias[3], float accel_bias[6]);

/* ======================== 应用程序辅助函数 ======================== */

/**
 * @brief 初始化I2C总线
 * @param scl_pin SCL引脚号
 * @param sda_pin SDA引脚号
 * @param freq_hz I2C频率(Hz)
 * @param bus_handle 输出I2C总线句柄
 * @return esp_err_t ESP_OK表示成功，其他表示错误
 */
esp_err_t mpu6500_init_i2c_bus(uint8_t scl_pin, uint8_t sda_pin, 
                               uint32_t freq_hz, i2c_master_bus_handle_t* bus_handle);

/**
 * @brief 验证MPU6500传感器连接
 * @param mpu MPU6500设备结构体指针
 * @return esp_err_t ESP_OK表示验证成功，其他表示错误
 */
esp_err_t mpu6500_verify_sensor(mpu6500_t* mpu);





#ifdef __cplusplus
}
#endif

#endif /* MPU6500_H */