/**
 * @file main.c
 * @brief MPU6500 usage example for ESP-IDF
 */

#include "mpu6500.h"
#include "driver/i2c_master.h"

static mpu6500_t mpu6500;
static i2c_master_bus_handle_t bus_handle;

#define EXAMPLE_SDA_PIN 8
#define EXAMPLE_SCL_PIN 18


void app_main(void) {
    esp_err_t ret;
    
    // I2C bus configuration
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = EXAMPLE_SDA_PIN,
        .scl_io_num = EXAMPLE_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    // Initialize I2C bus
    ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to initialize I2C bus");
        return;
    }
    
    // Initialize MPU6500 device
    ret = mpu6500_dev_init(bus_handle, &mpu6500, MPU6500_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to initialize MPU6500 device");
        return;
    }
    
    // Initialize MPU6500 sensor
    ret = mpu6500_sensor_init(&mpu6500);
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Failed to initialize MPU6500 sensor");
        return;
    }
    
    // Read WHO_AM_I register
    uint8_t whoami;
    ret = mpu6500_read_whoami(&mpu6500, &whoami);
    if (ret == ESP_OK) {
        ESP_LOGI("MAIN", "WHO_AM_I: 0x%02X", whoami);
    }
    
    // Main loop to read sensor data
    while (1) {
        int16_t accel_x, accel_y, accel_z;
        int16_t gyro_x, gyro_y, gyro_z;
        int16_t temp;
        
        // Read accelerometer
        if (mpu6500_read_accel(&mpu6500, &accel_x, &accel_y, &accel_z) == ESP_OK) {
            ESP_LOGI("MAIN", "Accel: X=%d, Y=%d, Z=%d", accel_x, accel_y, accel_z);
        }
        
        // Read gyroscope
        if (mpu6500_read_gyro(&mpu6500, &gyro_x, &gyro_y, &gyro_z) == ESP_OK) {
            ESP_LOGI("MAIN", "Gyro: X=%d, Y=%d, Z=%d", gyro_x, gyro_y, gyro_z);
        }
        
        // Read temperature
        if (mpu6500_read_temp(&mpu6500, &temp) == ESP_OK) {
            float temp_c = (float)temp / 333.87 + 21.0;
            ESP_LOGI("MAIN", "Temperature: %.2f°C", temp_c);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Cleanup (this part won't be reached in this example)
    mpu6500_deinit(&mpu6500);
    i2c_del_master_bus(bus_handle);
}