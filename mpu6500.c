/**
 * @file mpu6500.c
 * @brief MPU6500 accelerometer and gyroscope driver for ESP-IDF
 * @details This file contains the implementation of the
 *          MPU6500 accelerometer and gyroscope driver, supporting various
 *          operations.
 * @author Cengiz Sinan Kostakoglu
 * @version 1.0
 * @date 2025-06-08
 */

#include "mpu6500.h"


/* NVS存储相关定义 */
#define NVS_NAMESPACE "attitude_filter"
#define NVS_KEY_GYRO_BIAS "gyro_bias"
#define NVS_KEY_GYRO_CALIB_FLAG "gyro_cali"
#define NVS_KEY_ACCEL_BIAS "accel_bias"
#define NVS_KEY_ACCEL_SCALE "accel_scale"
#define NVS_KEY_ACCEL_CALIB_FLAG "a_statei"


static const char *TAG = "MPU6500";

static attitude_data_t attitude_data = { // 内部姿态数据
    .q0 = 1.0f,
    .q1 = 0.0f,
    .q2 = 0.0f,
    .q3 = 0.0f,
    .roll = 0.0f,
    .pitch = 0.0f,
    .yaw = 0.0f,
};

static sensor_read_config_t g_read_config = {  // 默认读取配置
    .accel_scale = 1.0f / ACCEL_SENSITIVITY,  // ±2g量程
    .gyro_scale = 1.0f / GYRO_SENSITIVITY,      // ±500°/s量程 (更新后)
    .use_calibration = true
};

static mpu_filter_config_t fliter_config = {    // 默认滤波器配置
    .sample_rate_hz = SAMPLE_RATE_HZ,
    .kp = 6.0,  // 比例增益
    .ki = 0.005f, // 积分增益
    .beta = 0.05f // 陀螺仪偏差估计增益
};

static calibration_data_t cali_data = { // 加速度计校准数据
    .accel_bias = {0.0f, 0.0f, 0.0f},
    .accel_scale = {1.0f, 1.0f, 1.0f},
    .accel_calibrated = false,
    .gyro_bias = {0.0f, 0.0f, 0.0f},
    .gyro_calibrated = false
};

static nvs_handle_t g_nvs_handle = 0; // NVS句柄

static float g_integral_fb[3] = {0.0f, 0.0f, 0.0f}; // 积分误差



/* ======================== MPU6500 内部寄存器操作函数 ======================== */

static esp_err_t mpu6500_write_register(mpu6500_t *mpu, uint8_t reg, uint8_t data)
{
    uint8_t write_buf[2] = {reg, data};
    return i2c_master_transmit(mpu->dev_handle, write_buf, sizeof(write_buf), -1);
}

static esp_err_t mpu6500_read_register(mpu6500_t *mpu, uint8_t reg, uint8_t *data)
{
    esp_err_t ret = i2c_master_transmit(mpu->dev_handle, &reg, 1, -1);
    if (ret != ESP_OK)
        return ret;
    return i2c_master_receive(mpu->dev_handle, data, 1, -1);
}

static esp_err_t mpu6500_read_registers(mpu6500_t *mpu, uint8_t reg, uint8_t *data, size_t len)
{
    esp_err_t ret = i2c_master_transmit(mpu->dev_handle, &reg, 1, -1);
    if (ret != ESP_OK)
        return ret;
    return i2c_master_receive(mpu->dev_handle, data, len, -1);
}

static esp_err_t mpu6500_reset(mpu6500_t *mpu)
{
    return mpu6500_write_register(mpu, PWR_MGMT_1, 0x80); // DEVICE_RESET[7]
}

static esp_err_t mpu6500_configure_clock(mpu6500_t *mpu)
{
    return mpu6500_write_register(mpu, PWR_MGMT_1, 0x01); // SLEEP[6] | CLKSEL[2:0]
}

static esp_err_t mpu6500_configure_accel(mpu6500_t *mpu)
{
    esp_err_t ret;

    // 配置加速度计量程范围 (±2g)
    ret = mpu6500_write_register(mpu, ACCEL_CONFIG, ACCEL_FS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure accelerometer full scale range");
        return ret;
    }

    // 配置加速度计低通滤波器 (44.8Hz, 1kHz)
    ret = mpu6500_write_register(mpu, ACCEL_CONFIG_2, 0x00);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure accelerometer low pass filter");
        return ret;
    }

    return ESP_OK;
}

static esp_err_t mpu6500_configure_gyro(mpu6500_t *mpu)
{
    esp_err_t ret;

    // 配置陀螺仪量程范围  (±500°/s)
    ret = mpu6500_write_register(mpu, GYRO_CONFIG, GYRO_FS);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure gyroscope full scale range");
        return ret;
    }

    // 配置陀螺仪低通滤波器 (44.8Hz, 1kHz)
    ret = mpu6500_write_register(mpu, CONFIG, 0x00);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure gyroscope low pass filter");
        return ret;
    }

    return ESP_OK;
}

static esp_err_t mpu6500_enable_temperature_sensor(mpu6500_t *mpu)
{
    esp_err_t ret;
    uint8_t reg_data;

    ret = mpu6500_read_register(mpu, PWR_MGMT_1, &reg_data);
    if (ret != ESP_OK)
        return ret;

    // 清除TEMP_DIS位 (bit 4)
    reg_data &= ~(1 << 4);

    return mpu6500_write_register(mpu, PWR_MGMT_1, reg_data);
}

static esp_err_t mpu6500_configure_interrupts(mpu6500_t *mpu)
{
    return mpu6500_write_register(mpu, INT_PIN_CFG, 0xB0); // ACTL[7] | OPEN[6] | LATCH_INT_EN[5] | INT_ANYRD_2CLEAR[4]
}

/* ======================== MPU6500 公共驱动函数实现 ======================== */

esp_err_t mpu6500_dev_init(i2c_master_bus_handle_t bus_handle, mpu6500_t *mpu, uint8_t i2c_addr)
{
    if (mpu == NULL || bus_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = 400000,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &mpu->dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add MPU6500 device to I2C bus");
        return ret;
    }

    mpu->i2c_addr = i2c_addr;
    ESP_LOGI(TAG, "MPU6500 device initialized with address 0x%02X", i2c_addr);

    return ESP_OK;
}

esp_err_t mpu6500_sensor_init(mpu6500_t *mpu)
{
    if (mpu == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    // 1. 复位设备
    ret = mpu6500_reset(mpu);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to reset MPU6500");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // 等待复位完成

    // 2. 唤醒设备并选择时钟源
    ret = mpu6500_configure_clock(mpu);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure MPU6500 clock");
        return ret;
    }

    // 3. 配置加速度计
    ret = mpu6500_configure_accel(mpu);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure accelerometer");
        return ret;
    }

    // 4. 配置陀螺仪
    ret = mpu6500_configure_gyro(mpu);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure gyroscope");
        return ret;
    }

    // 5. 使能温度传感器
    ret = mpu6500_enable_temperature_sensor(mpu);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable temperature sensor");
        return ret;
    }

    // 6. 配置INT引脚
    ret = mpu6500_configure_interrupts(mpu);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure interrupts");
        return ret;
    }

    ESP_LOGI(TAG, "MPU6500 sensor initialized successfully");
    return ESP_OK;
}

esp_err_t mpu6500_read_whoami(mpu6500_t *mpu, uint8_t *whoami)
{
    if (mpu == NULL || whoami == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }
    return mpu6500_read_register(mpu, WHO_AM_I, whoami);
}

esp_err_t mpu6500_read_accel(mpu6500_t *mpu, int16_t *x, int16_t *y, int16_t *z)
{
    if (mpu == NULL || x == NULL || y == NULL || z == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[6];
    esp_err_t ret = mpu6500_read_registers(mpu, ACCEL_XOUT_H, buffer, sizeof(buffer));
    if (ret != ESP_OK)
    {
        return ret;
    }

    *x = (int16_t)((buffer[0] << 8) | buffer[1]);
    *y = (int16_t)((buffer[2] << 8) | buffer[3]);
    *z = (int16_t)((buffer[4] << 8) | buffer[5]);

    return ESP_OK;
}

esp_err_t mpu6500_read_gyro(mpu6500_t *mpu, int16_t *x, int16_t *y, int16_t *z)
{
    if (mpu == NULL || x == NULL || y == NULL || z == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[6];
    esp_err_t ret = mpu6500_read_registers(mpu, GYRO_XOUT_H, buffer, sizeof(buffer));
    if (ret != ESP_OK)
    {
        return ret;
    }

    *x = (int16_t)((buffer[0] << 8) | buffer[1]);
    *y = (int16_t)((buffer[2] << 8) | buffer[3]);
    *z = (int16_t)((buffer[4] << 8) | buffer[5]);

    return ESP_OK;
}

esp_err_t mpu6500_read_temp(mpu6500_t *mpu, int16_t *temp)
{
    if (mpu == NULL || temp == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t buffer[2];
    esp_err_t ret = mpu6500_read_registers(mpu, TEMP_OUT_H, buffer, sizeof(buffer));
    if (ret != ESP_OK)
    {
        return ret;
    }

    *temp = (int16_t)((buffer[0] << 8) | buffer[1]);
    return ESP_OK;
}

esp_err_t mpu6500_read_sensor_data(mpu6500_t *mpu, const sensor_read_config_t *config,
                                   float accel_g[3], float gyro_dps[3])
{
    if (mpu == NULL || accel_g == NULL || gyro_dps == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    int16_t accel_raw[3], gyro_raw[3];
    esp_err_t ret;

    // 读取加速度计原始数据
    ret = mpu6500_read_accel(mpu, &accel_raw[0], &accel_raw[1], &accel_raw[2]);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "读取加速度计数据失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 读取陀螺仪原始数据
    ret = mpu6500_read_gyro(mpu, &gyro_raw[0], &gyro_raw[1], &gyro_raw[2]);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "读取陀螺仪数据失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 转换为物理单位
    float accel_scale = (config != NULL) ? config->accel_scale : g_read_config.accel_scale; // 默认±2g量程
    accel_g[0] = accel_raw[0] * accel_scale;
    accel_g[1] = accel_raw[1] * accel_scale;
    accel_g[2] = accel_raw[2] * accel_scale;

    // 应用加速度计校准
    if (cali_data.accel_calibrated && (config == NULL || config->use_calibration))
    {
        // 先减去零偏，再乘以标度因子
        accel_g[0] = (accel_g[0] - cali_data.accel_bias[0]) * cali_data.accel_scale[0];
        accel_g[1] = (accel_g[1] - cali_data.accel_bias[1]) * cali_data.accel_scale[1];
        accel_g[2] = (accel_g[2] - cali_data.accel_bias[2]) * cali_data.accel_scale[2];
    }

    // 陀螺仪转换
    float gyro_scale = (config != NULL) ? config->gyro_scale : g_read_config.gyro_scale; // 默认±500°/s量程
    gyro_dps[0] = gyro_raw[0] * gyro_scale;
    gyro_dps[1] = gyro_raw[1] * gyro_scale;
    gyro_dps[2] = gyro_raw[2] * gyro_scale;

    return ESP_OK;
}

esp_err_t mpu6500_enable_data_ready_interrupts(mpu6500_t *mpu)
{
    return mpu6500_write_register(mpu, INT_ENABLE, 0x01); // RAW_RDY_EN[0]
}

esp_err_t mpu6500_disable_data_ready_interrupts(mpu6500_t *mpu)
{
    return mpu6500_write_register(mpu, INT_ENABLE, 0x00); // RAW_RDY_EN[0]
}

esp_err_t mpu6500_sleep(mpu6500_t *mpu)
{
    esp_err_t ret;
    uint8_t reg_data;

    ret = mpu6500_read_register(mpu, PWR_MGMT_1, &reg_data);
    if (ret != ESP_OK)
        return ret;

    // 设置SLEEP位 (bit 6)
    reg_data |= (1 << 6);

    return mpu6500_write_register(mpu, PWR_MGMT_1, reg_data);
}

esp_err_t mpu6500_wakeup(mpu6500_t *mpu)
{
    esp_err_t ret;
    uint8_t reg_data;

    ret = mpu6500_read_register(mpu, PWR_MGMT_1, &reg_data);
    if (ret != ESP_OK)
        return ret;

    // 清除SLEEP位 (bit 6)
    reg_data &= ~(1 << 6);

    return mpu6500_write_register(mpu, PWR_MGMT_1, reg_data);
}

esp_err_t mpu6500_deinit(mpu6500_t *mpu)
{
    if (mpu == NULL || mpu->dev_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_master_bus_rm_device(mpu->dev_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to remove MPU6500 device from I2C bus");
        return ret;
    }

    mpu->dev_handle = NULL;
    ESP_LOGI(TAG, "MPU6500 device deinitialized");

    return ESP_OK;
}

/* ======================== 姿态滤波器内部函数 ======================== */

static void mahony_filter_update(const float accel[3], const float gyro[3], float dt)
{
    float q0 = attitude_data.q0, q1 = attitude_data.q1, q2 = attitude_data.q2, q3 = attitude_data.q3;

    // 归一化加速度计
    float norm = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if (norm < 0.001f)
        return; // 避免除零

    float ax = accel[0] / norm;
    float ay = accel[1] / norm;
    float az = accel[2] / norm;

    // 估计重力和磁力方向（这里简化，只使用重力）
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 计算误差（向量叉积）
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    // 积分误差
    if (fliter_config.ki > 0.0f)
    {
        g_integral_fb[0] += ex * fliter_config.ki * dt;
        g_integral_fb[1] += ey * fliter_config.ki * dt;
        g_integral_fb[2] += ez * fliter_config.ki * dt;

        // 积分限幅
        const float integral_limit = 0.1f;
        for (int i = 0; i < 3; i++)
        {
            if (g_integral_fb[i] > integral_limit)
                g_integral_fb[i] = integral_limit;
            if (g_integral_fb[i] < -integral_limit)
                g_integral_fb[i] = -integral_limit;
        }
    }

    // 应用反馈
    float gx = gyro[0] + fliter_config.kp * ex + g_integral_fb[0];
    float gy = gyro[1] + fliter_config.kp * ey + g_integral_fb[1];
    float gz = gyro[2] + fliter_config.kp * ez + g_integral_fb[2];

    // 转换为弧度/秒
    gx *= (M_PI / 180.0f);
    gy *= (M_PI / 180.0f);
    gz *= (M_PI / 180.0f);

    // 四元数积分
    float q0_temp = q0 + 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
    float q1_temp = q1 + 0.5f * (q0 * gx + q2 * gz - q3 * gy) * dt;
    float q2_temp = q2 + 0.5f * (q0 * gy - q1 * gz + q3 * gx) * dt;
    float q3_temp = q3 + 0.5f * (q0 * gz + q1 * gy - q2 * gx) * dt;

    // 归一化四元数
    norm = sqrtf(q0_temp * q0_temp + q1_temp * q1_temp + q2_temp * q2_temp + q3_temp * q3_temp);
    if (norm > 0.0f)
    {
        q0_temp /= norm;
        q1_temp /= norm;
        q2_temp /= norm;
        q3_temp /= norm;
    }

    // 更新状态
    attitude_data.q0 = q0_temp;
    attitude_data.q1 = q1_temp;
    attitude_data.q2 = q2_temp;
    attitude_data.q3 = q3_temp;
}

static void update_euler_angles(void)
{
    float q0 = attitude_data.q0, q1 = attitude_data.q1, q2 = attitude_data.q2, q3 = attitude_data.q3;

    // 横滚角 (x-axis rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    attitude_data.roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / M_PI);

    // 俯仰角 (y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f)
    {
        attitude_data.pitch = copysignf(M_PI / 2.0f, sinp) * (180.0f / M_PI);
    }
    else
    {
        attitude_data.pitch = asinf(sinp) * (180.0f / M_PI);
    }

    // 航向角 (z-axis rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    attitude_data.yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI);
}

static float calculate_confidence(const float accel[3], const float gyro[3])
{
    float confidence = 1.0f;

    // 检查加速度计量级（应在1g附近）
    float accel_mag = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    float accel_error = fabsf(accel_mag - 1.0f);
    if (accel_error > 0.3f)
    {
        confidence *= (1.0f - accel_error);
    }

    // 检查角速度是否在合理范围内
    for (int i = 0; i < 3; i++)
    {
        if (fabsf(gyro[i]) > 500.0f)
        {
            confidence *= 0.7f;
        }
    }

    return (confidence < 0.1f) ? 0.1f : confidence;
}

/* ======================== 姿态滤波器公共函数实现 ======================== */

void attitude_filter_init(const mpu_filter_config_t *config)
{
    if (config != NULL)
    {
        fliter_config = *config;
    }
    esp_err_t load_err = mpu6500_load_gyro_calibration();

    if (load_err == ESP_OK)
    {
        // 成功加载，使用保存的校准数据
        ESP_LOGI(TAG, "使用NVS中保存的校准参数");
    }
    else
    {
        // 加载失败，重置状态，等待校准
        attitude_filter_reset();
        ESP_LOGI(TAG, "未找到有效校准数据，等待传感器校准");
    }

    ESP_LOGI(TAG, "姿态滤波器初始化完成");
    ESP_LOGI(TAG, "采样率: %.0f Hz, KP: %.3f, KI: %.3f",
             fliter_config.sample_rate_hz, fliter_config.kp, fliter_config.ki);
}

void attitude_filter_update(const float accel[3], const float gyro[3],
                            float dt, attitude_data_t *output)
{
    if (output == NULL)
        return;

    // 校准阶段：估计陀螺仪零偏
    if (!cali_data.gyro_calibrated)
    {
        ESP_LOGW(TAG, "姿态滤波器未校准!!!");
    }

    // 补偿零偏的陀螺仪数据
    float gyro_corrected[3] = {
        gyro[0] - cali_data.gyro_bias[0],
        gyro[1] - cali_data.gyro_bias[1],
        gyro[2] - cali_data.gyro_bias[2]};

    // 保存角速度
    attitude_data.gyro_x = gyro_corrected[0];
    attitude_data.gyro_y = gyro_corrected[1];
    attitude_data.gyro_z = gyro_corrected[2];

    // Mahony滤波器更新
    mahony_filter_update(accel, gyro_corrected, dt);

    // 更新欧拉角
    update_euler_angles();

    // 复制输出
    *output = attitude_data;
    output->timestamp_us = esp_timer_get_time();
    output->confidence = calculate_confidence(accel, gyro_corrected);
}

void attitude_filter_reset(void)
{
    attitude_data.q0 = 1.0f;
    attitude_data.q1 = 0.0f;
    attitude_data.q2 = 0.0f;
    attitude_data.q3 = 0.0f;
    attitude_data.roll = 0.0f;
    attitude_data.pitch = 0.0f;
    attitude_data.yaw = 0.0f;
    cali_data.gyro_calibrated = false;
    g_integral_fb[0] = g_integral_fb[1] = g_integral_fb[2] = 0.0f;

    for (int i = 0; i < 3; i++)
    {
        cali_data.gyro_bias[i] = 0.0f;
    }
}

bool attitude_filter_is_calibrated(void)
{
    return cali_data.gyro_calibrated;
}

/* ======================== NVS存储管理函数实现 ======================== */

esp_err_t attitude_filter_nvs_init(void)
{
    esp_err_t err = nvs_flash_init();

    // 处理常见的NVS初始化错误（分区满或版本不匹配）
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGW(TAG, "NVS分区需要擦除,正在重新初始化...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // 打开（或创建）我们专用的命名空间
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &g_nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "打开NVS命名空间失败: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "NVS初始化成功");
    return ESP_OK;
}

esp_err_t attitude_filter_clear_nvs_calibration(void){

    if (g_nvs_handle == 0){
        return ESP_FAIL;
    }
    esp_err_t err;

    // 删除校准数据键
    err = nvs_erase_key(g_nvs_handle, NVS_KEY_ACCEL_BIAS);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND){
        ESP_LOGE(TAG, "删除加速度偏键失败: %s", esp_err_to_name(err));
    }

    err = nvs_erase_key(g_nvs_handle, NVS_KEY_GYRO_BIAS);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND){
        ESP_LOGE(TAG, "删除陀螺仪零偏键失败: %s", esp_err_to_name(err));
    }

    err = nvs_erase_key(g_nvs_handle, NVS_KEY_ACCEL_SCALE);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND){
        ESP_LOGE(TAG, "删除加速度尺度因子失败: %s", esp_err_to_name(err));
    }
    // 删除校准标志键
    err = nvs_erase_key(g_nvs_handle, NVS_KEY_GYRO_CALIB_FLAG);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND){
        ESP_LOGE(TAG, "删除校准陀螺仪标志键失败: %s", esp_err_to_name(err));
    }

    err = nvs_erase_key(g_nvs_handle, NVS_KEY_ACCEL_CALIB_FLAG);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND){
        ESP_LOGE(TAG, "删除校准加速度标志键失败: %s", esp_err_to_name(err));
    }
    // 提交更改
    err = nvs_commit(g_nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "提交清除操作失败: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "NVS中的校准数据已清除");
    return ESP_OK;
}

/* ======================== 加速度计校准函数实现 ======================== */

esp_err_t mpu6500_calibrate_accel(mpu6500_t *mpu, const sensor_read_config_t *config)
{
    if (mpu == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "开始加速度计校准...");
    ESP_LOGI(TAG, "步骤1: 请将设备水平放置(Z轴朝上)保持3秒");

    // 阶段1: 水平放置，Z轴朝上
    float sum_z_up[3] = {0};
    uint32_t samples = 0;
    const uint32_t total_samples = 750;
    float accel_scale = (config != NULL) ? config->accel_scale : (1.0f / ACCEL_SENSITIVITY);

    while (samples < total_samples)
    {
        int16_t accel_raw[3];
        if (samples % 10 == 0)
        {
            ESP_LOGI(TAG, "当前样本数: %u ,进度 %f", samples, (float)samples / total_samples * 100.0f);
        }
        
        if (mpu6500_read_accel(mpu, &accel_raw[0], &accel_raw[1], &accel_raw[2]) == ESP_OK)
        {
            sum_z_up[0] += accel_raw[0] * accel_scale;
            sum_z_up[1] += accel_raw[1] * accel_scale;
            sum_z_up[2] += accel_raw[2] * accel_scale;
            samples++;
        vTaskDelay(pdMS_TO_TICKS(4)); // 采样间隔约为4ms
        }

    }

    float avg_z_up[3] = {
        sum_z_up[0] / total_samples,
        sum_z_up[1] / total_samples,
        sum_z_up[2] / total_samples
    };

    ESP_LOGI(TAG, "步骤1完成: [%.4f, %.4f, %.4f] g", 
             avg_z_up[0], avg_z_up[1], avg_z_up[2]);

    // 阶段2: 水平放置，Z轴朝下
    ESP_LOGI(TAG, "步骤2: 请将设备翻转180度(Z轴朝下)保持3秒");
    ESP_LOGI(TAG, "等待4秒后开始...");
    vTaskDelay(pdMS_TO_TICKS(4000));
    
    samples = 0;
    float sum_z_down[3] = {0};
    
    while (samples < total_samples)
    {
        int16_t accel_raw[3];
        if (samples % 10 == 0)
        {
            ESP_LOGI(TAG, "当前样本数: %u ,进度 %f", samples, (float)samples / total_samples * 100.0f);
        }
        
        if (mpu6500_read_accel(mpu, &accel_raw[0], &accel_raw[1], &accel_raw[2]) == ESP_OK)
        {
            sum_z_down[0] += accel_raw[0] * accel_scale;
            sum_z_down[1] += accel_raw[1] * accel_scale;
            sum_z_down[2] += accel_raw[2] * accel_scale;
            samples++;
        vTaskDelay(pdMS_TO_TICKS(4)); // 采样间隔约为4ms
        }

    }

    float avg_z_down[3] = {
        sum_z_down[0] / total_samples,
        sum_z_down[1] / total_samples,
        sum_z_down[2] / total_samples
    };

    ESP_LOGI(TAG, "步骤2完成: [%.4f, %.4f, %.4f] g", 
             avg_z_down[0], avg_z_down[1], avg_z_down[2]);

    // 计算零偏和标度因子
    // 理想情况：+Z: [0, 0, +1], -Z: [0, 0, -1]
    
    // 计算零偏：两个方向的平均值
    cali_data.accel_bias[0] = (avg_z_up[0] + avg_z_down[0]) / 2.0f;
    cali_data.accel_bias[1] = (avg_z_up[1] + avg_z_down[1]) / 2.0f;
    cali_data.accel_bias[2] = (avg_z_up[2] + avg_z_down[2]) / 2.0f;
    
    // 计算标度因子：(实际测量值 - 零偏) 应等于理论值
    // 对于Z轴：+Z时理论值为+1g，-Z时理论值为-1g
    float z_measured_range_up = avg_z_up[2] - cali_data.accel_bias[2];   // 应接近 +1
    float z_measured_range_down = avg_z_down[2] - cali_data.accel_bias[2]; // 应接近 -1
    
    // 计算实际的幅度
    float z_range_measured = fabsf(z_measured_range_up - z_measured_range_down) / 2.0f;
    // 理论幅度应为 2g（从+1到-1）
    float z_scale_factor = 1.0f / z_range_measured;
    
    // 使用相同的标度因子假设XY轴（简化，实际可能需要单独校准）
    cali_data.accel_scale[0] = z_scale_factor;
    cali_data.accel_scale[1] = z_scale_factor;
    cali_data.accel_scale[2] = z_scale_factor;
    
    cali_data.accel_calibrated = true;

    // 保存到NVS
    esp_err_t save_err = mpu6500_save_accel_calibration();

    ESP_LOGI(TAG, "加速度计校准完成:");
    ESP_LOGI(TAG, "  零偏: X=%.6f, Y=%.6f, Z=%.6f g",
             cali_data.accel_bias[0], cali_data.accel_bias[1], cali_data.accel_bias[2]);
    ESP_LOGI(TAG, "  标度因子: X=%.6f, Y=%.6f, Z=%.6f",
             cali_data.accel_scale[0], cali_data.accel_scale[1], cali_data.accel_scale[2]);
    ESP_LOGI(TAG, "  Z轴测量范围: %.6f g", z_range_measured);
    
    // 验证校准效果
    ESP_LOGI(TAG, "校准验证:");
    ESP_LOGI(TAG, "  +Z方向(校准后): %.4f g", 
             (avg_z_up[2] - cali_data.accel_bias[2]) * cali_data.accel_scale[2]);
    ESP_LOGI(TAG, "  -Z方向(校准后): %.4f g", 
             (avg_z_down[2] - cali_data.accel_bias[2]) * cali_data.accel_scale[2]);

    if (save_err == ESP_OK)
    {
        ESP_LOGI(TAG, "校准数据已保存到NVS");
    }

    return ESP_OK;
}

esp_err_t mpu6500_calibrate_gyro(mpu6500_t *mpu,const sensor_read_config_t *config)
{
    if (mpu == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "开始简化的陀螺仪零偏校准...");
    ESP_LOGI(TAG, "请将设备保持静止,保持3秒");


    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    uint32_t samples = 0;
    const uint32_t total_samples = 750;

    float gyro_scale = (config != NULL) ? config->gyro_scale : (1.0f / GYRO_SENSITIVITY); // 默认±500°/s

    while (samples < total_samples)
    {
        int16_t gyro_raw[3];
        
        // 显示进度
        if (samples % 10 == 0)
        {
            ESP_LOGI(TAG, "当前样本数: %u ,进度 %f", samples, (float)samples / total_samples * 100.0f);
        }
        
        // 读取陀螺仪原始数据
        if (mpu6500_read_gyro(mpu, &gyro_raw[0], &gyro_raw[1], &gyro_raw[2]) == ESP_OK)
        {
            // 转换为度/秒单位
            gyro_sum[0] += gyro_raw[0] * gyro_scale;
            gyro_sum[1] += gyro_raw[1] * gyro_scale;
            gyro_sum[2] += gyro_raw[2] * gyro_scale;
            samples++;
        vTaskDelay(pdMS_TO_TICKS(4)); // 采样间隔约为4ms
        }

        
    }

    // 计算平均值作为零偏
    cali_data.gyro_bias[0] = gyro_sum[0] / total_samples;
    cali_data.gyro_bias[1] = gyro_sum[1] / total_samples;
    cali_data.gyro_bias[2] = gyro_sum[2] / total_samples;
    cali_data.gyro_calibrated = true;

    // 保存到NVS
    esp_err_t save_err = mpu6500_save_gyro_calibration();

    ESP_LOGI(TAG, "陀螺仪零偏校准完成:");
    ESP_LOGI(TAG, "  零偏: X=%.3f, Y=%.3f, Z=%.3f °/s",
             cali_data.gyro_bias[0], cali_data.gyro_bias[1], cali_data.gyro_bias[2]);
             
    if (save_err == ESP_OK)
    {
        ESP_LOGI(TAG, "校准数据已保存到NVS");
    }
    else
    {
        ESP_LOGW(TAG, "校准完成,但保存到NVS失败: %s", esp_err_to_name(save_err));
    }

    return ESP_OK;
}

esp_err_t mpu6500_set_calibration(const float gyro_bias[3], const float accel_bias[6])
{
    if (gyro_bias == NULL)
    {
        return ESP_ERR_INVALID_ARG;
        ESP_LOGE(TAG, "无效的陀螺仪校准参数");
    }
        cali_data.gyro_bias[0] = gyro_bias[0];
        cali_data.gyro_bias[1] = gyro_bias[1];
        cali_data.gyro_bias[2] = gyro_bias[2];
        cali_data.gyro_calibrated = true;
        mpu6500_save_gyro_calibration();
    
        if (accel_bias == NULL)
    {
        return ESP_ERR_INVALID_ARG;
        ESP_LOGE(TAG, "无效的加速度计校准参数");
    }
        cali_data.accel_bias[0] = gyro_bias[0];
        cali_data.accel_bias[1] = gyro_bias[1];
        cali_data.accel_bias[2] = gyro_bias[2];
        cali_data.accel_scale[3] = accel_bias[4];
        cali_data.accel_scale[4] = accel_bias[5];
        cali_data.accel_scale[5] = accel_bias[6];
        cali_data.accel_calibrated = true;
        mpu6500_save_accel_calibration();
    
    return ESP_OK;
}

esp_err_t mpu6500_get_calibration(float gyro_bias[3], float accel_bias[6])
{
    if (gyro_bias == NULL || !cali_data.gyro_calibrated)
    {
        return ESP_ERR_INVALID_ARG;
        ESP_LOGE(TAG, "存储数组无效或陀螺仪未校准");
    }
        gyro_bias[0] = cali_data.gyro_bias[0];
        gyro_bias[1] = cali_data.gyro_bias[1];
        gyro_bias[2] = cali_data.gyro_bias[2];
    
    if (accel_bias == NULL || !cali_data.accel_calibrated)
    {
        return ESP_ERR_INVALID_ARG;
        ESP_LOGE(TAG, "存储数组无效或加速度计未校准");
    }
        accel_bias[0] = cali_data.accel_bias[0];
        accel_bias[1] = cali_data.accel_bias[1];
        accel_bias[2] = cali_data.accel_bias[2];
        accel_bias[3] = cali_data.accel_scale[3];
        accel_bias[4] = cali_data.accel_scale[4];
        accel_bias[5] = cali_data.accel_scale[5];
    
    return ESP_OK;
}

esp_err_t mpu6500_save_gyro_calibration(void)
{
    if (g_nvs_handle == 0)
    {
        ESP_LOGE(TAG, "NVS句柄未初始化,无法保存");
        return ESP_FAIL;
    }

    esp_err_t err;

    // 保存陀螺仪零偏（二进制BLOB数据）
    err = nvs_set_blob(g_nvs_handle, NVS_KEY_GYRO_BIAS,
                       cali_data.gyro_bias, sizeof(cali_data.gyro_bias));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "保存零偏数据失败: %s", esp_err_to_name(err));
        return err;
    }

    // 保存校准标志
    uint8_t calibrated = 1; // 用1表示已校准
    err = nvs_set_u8(g_nvs_handle, NVS_KEY_GYRO_CALIB_FLAG, calibrated);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "保存校准标志失败: %s", esp_err_to_name(err));
        return err;
    }

    // 关键步骤：提交更改到Flash
    err = nvs_commit(g_nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "提交NVS数据失败: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "校准数据已保存到NVS");
    return ESP_OK;
}

esp_err_t mpu6500_load_gyro_calibration(void)
{
    if (g_nvs_handle == 0)
    {
        ESP_LOGE(TAG, "NVS句柄未初始化,无法加载");
        return ESP_FAIL;
    }
    esp_err_t err;

    // 首先检查校准标志是否存在
    uint8_t calibrated = 0;
    err = nvs_get_u8(g_nvs_handle, NVS_KEY_GYRO_CALIB_FLAG, &calibrated);

    if (err == ESP_ERR_NVS_NOT_FOUND){
        ESP_LOGI(TAG, "未找到保存的校准数据，需要重新校准");
        return ESP_ERR_NVS_NOT_FOUND;
    }else if (err != ESP_OK){
        ESP_LOGE(TAG, "读取校准标志失败: %s", esp_err_to_name(err));
        return err;
    }

    if (calibrated != 1)
    {
        ESP_LOGW(TAG, "校准标志无效，需要重新校准");
        return ESP_FAIL;
    }

    // 读取陀螺仪零偏数据
    size_t required_size = sizeof(cali_data.gyro_bias);
    err = nvs_get_blob(g_nvs_handle, NVS_KEY_GYRO_BIAS,
                       cali_data.gyro_bias, &required_size);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "读取零偏数据失败: %s", esp_err_to_name(err));
        return err;
    }

    if (required_size != sizeof(cali_data.gyro_bias))
    {
        ESP_LOGW(TAG, "零偏数据大小不匹配，需要重新校准");
        return ESP_FAIL;
    }

    // 设置校准状态
    cali_data.gyro_calibrated = true;
    ESP_LOGI(TAG, "从NVS加载校准数据成功");

    // 打印加载的零偏值用于验证
    ESP_LOGI(TAG, "加载的陀螺仪零偏: [%.3f, %.3f, %.3f] °/s",
             cali_data.gyro_bias[0], cali_data.gyro_bias[1], cali_data.gyro_bias[2]);

    return ESP_OK;
}

esp_err_t mpu6500_save_accel_calibration(void)
{
    if (g_nvs_handle == 0)
    {
        ESP_LOGE(TAG, "NVS句柄未初始化,无法保存");
        return ESP_FAIL;
    }

    esp_err_t err;

    // 保存零偏
    err = nvs_set_blob(g_nvs_handle, NVS_KEY_ACCEL_BIAS,
                       cali_data.accel_bias, sizeof(cali_data.accel_bias));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "保存加速度计零偏失败: %s", esp_err_to_name(err));
        return err;
    }

    // 保存标度因子
    err = nvs_set_blob(g_nvs_handle, NVS_KEY_ACCEL_SCALE,
                       cali_data.accel_scale, sizeof(cali_data.accel_scale));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "保存加速度计标度因子失败: %s", esp_err_to_name(err));
        return err;
    }

    // 保存校准标志
    uint8_t cal_flag = cali_data.accel_calibrated ? 1 : 0;
    err = nvs_set_u8(g_nvs_handle, NVS_KEY_ACCEL_CALIB_FLAG, cal_flag);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "保存加速度计校准标志失败: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_commit(g_nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "提交加速度计校准数据失败: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "加速度计校准数据已保存到NVS");
    return ESP_OK;
}

esp_err_t mpu6500_load_accel_calibration(void)
{
    if (g_nvs_handle == 0)
    {
        ESP_LOGE(TAG, "NVS句柄未初始化,无法加载");
        return ESP_FAIL;
    }

    esp_err_t err;

    // 检查校准标志
    uint8_t calibrated = 0;
    err = nvs_get_u8(g_nvs_handle, NVS_KEY_ACCEL_CALIB_FLAG, &calibrated);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        return ESP_ERR_NVS_NOT_FOUND;
    }

    if (calibrated != 1)
    {
        return ESP_FAIL;
    }

    // 加载零偏
    size_t size = sizeof(cali_data.accel_bias);
    err = nvs_get_blob(g_nvs_handle, NVS_KEY_ACCEL_BIAS, cali_data.accel_bias, &size);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "加载加速度计零偏失败: %s", esp_err_to_name(err));
        return err;
    }

    // 加载标度因子
    size = sizeof(cali_data.accel_scale);
    err = nvs_get_blob(g_nvs_handle, NVS_KEY_ACCEL_SCALE, cali_data.accel_scale, &size);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "加载加速度计标度因子失败: %s", esp_err_to_name(err));
        return err;
    }

    cali_data.accel_calibrated = true;

    ESP_LOGI(TAG, "从NVS加载加速度计校准数据成功");
    ESP_LOGI(TAG, "零偏: [%.4f, %.4f, %.4f] g",
             cali_data.accel_bias[0], cali_data.accel_bias[1], cali_data.accel_bias[2]);

    return ESP_OK;
}

/* ======================== 应用程序辅助函数实现 ======================== */

esp_err_t mpu6500_init_i2c_bus(uint8_t scl_pin, uint8_t sda_pin,
                               uint32_t freq_hz, i2c_master_bus_handle_t *bus_handle)
{
    if (bus_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "初始化I2C总线: SCL=GPIO%d, SDA=GPIO%d, 频率=%dHz",
             scl_pin, sda_pin, freq_hz);

    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = scl_pin,
        .sda_io_num = sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, bus_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "创建I2C总线失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C总线初始化成功");
    return ESP_OK;
}

esp_err_t mpu6500_verify_sensor(mpu6500_t *mpu)
{
    if (mpu == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "验证MPU6500传感器...");

    // 读取WHO_AM_I寄存器
    uint8_t whoami = 0;
    esp_err_t ret = mpu6500_read_whoami(mpu, &whoami);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "读取WHO_AM_I失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // MPU6500的WHO_AM_I值应为0x70
    if (whoami != 0x70)
    {
        ESP_LOGE(TAG, "WHO_AM_I值错误: 0x%02X (期望: 0x70)", whoami);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "MPU6500验证成功, WHO_AM_I: 0x%02X", whoami);
    return ESP_OK;
}

