/*/**
 * @file advanced_example.c
 * @brief MPU6500传感器验证程序
 * @details 用于验证MPU6500驱动库的所有功能
 * @author  钠的水溶液
 * @version 1.0
 * @date 2025-06-09
 */


//如果出现传感器校准时间错误请修改menuconfig 中的configTICK_RATE_HZ为1000，
//因为程序中使用了vTaskDelay(pdMS_TO_TICKS(x))函数进行延时，configTICK_RATE_HZ默认为100
//这会导致毫秒延迟不准确，从而影响校准时间

//如果出现角度不准请修改此项,位于头文件中，非常重要！！！滤波器工作需要一个精确的dt，而每个人的系统频率不一样
//#define SAMPLE_RATE_HZ  80.0f 
//出现超调震荡等请自行修改 mpu_filter_config_t fliter_config 中的kp,ki,beta参数



#include "mpu6500.h"
#include "driver/uart.h"
#include <stdio.h>
#include <string.h>

// 默认配置
#define I2C_SCL_PIN     14
#define I2C_SDA_PIN     13
#define I2C_FREQ_HZ     400000




// UART配置
#define UART_NUM        UART_NUM_0
#define BUF_SIZE        1024
#define RD_BUF_SIZE     128

// 全局变量
static const char *TAG = "MAIN";
static mpu6500_t g_mpu;
static i2c_master_bus_handle_t g_bus_handle = NULL;
static output_mode_t g_output_mode = OUTPUT_MODE_DETAILED;


// 函数声明
static void print_sensor_data(const float accel[3], const float gyro[3]);
static void print_attitude_data(const attitude_data_t *attitude);
static void main_task(void *pvParameters);
static void calibrate_sensors(void);
static void verify_sensor_functionality(void);
static void run_continuous_data_acquisition(void);
static int get_char_nonblocking(void);
static void wait_for_any_key(void);
static void init_uart(void);
static int get_char_nonblocking(void);
static void wait_for_any_key(void);


void app_main(void)
{
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "MPU6500 传感器验证程序");
    ESP_LOGI(TAG, "版本: 1.0");
    ESP_LOGI(TAG, "编译日期: %s %s", __DATE__, __TIME__);
    ESP_LOGI(TAG, "=========================================");

    // 1. 初始化UART
    ESP_LOGI(TAG, "步骤1: 初始化UART...");
    init_uart();

    // 2. 初始化NVS存储
    ESP_LOGI(TAG, "步骤2: 初始化NVS存储...");
    esp_err_t ret = attitude_filter_nvs_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS初始化失败: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "NVS初始化成功");

    // 3. 初始化I2C总线
    ESP_LOGI(TAG, "步骤3: 初始化I2C总线...");
    ret = mpu6500_init_i2c_bus(I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ_HZ, &g_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C总线初始化失败: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "I2C总线初始化成功");

    // 4. 初始化MPU6500设备
    ESP_LOGI(TAG, "步骤4: 初始化MPU6500设备...");
    ret = mpu6500_dev_init(g_bus_handle, &g_mpu, MPU6500_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6500设备初始化失败: %s", esp_err_to_name(ret));
        return;
    }

    // 5. 初始化传感器配置
    ESP_LOGI(TAG, "步骤5: 初始化传感器配置...");
    ret = mpu6500_sensor_init(&g_mpu);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "传感器配置失败: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "传感器配置成功");

    // 6. 验证传感器连接
    ESP_LOGI(TAG, "步骤6: 验证传感器连接...");
    ret = mpu6500_verify_sensor(&g_mpu);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "传感器验证失败");
        return;
    }
    ESP_LOGI(TAG, "传感器验证成功");

    // 7. 初始化姿态滤波器
    ESP_LOGI(TAG, "步骤7: 初始化姿态滤波器...");

    attitude_filter_init(NULL);
    ESP_LOGI(TAG, "姿态滤波器初始化成功");

    // 8. 显示欢迎信息和命令帮助
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "系统初始化完成!");
    ESP_LOGI(TAG, "按以下按键进行交互:");
    ESP_LOGI(TAG, "  1. 'h' - 显示此界面信息");
    ESP_LOGI(TAG, "  2. 'c' - 执行传感器校准");
    ESP_LOGI(TAG, "  3. 't' - 测试基本功能");
    ESP_LOGI(TAG, "  4. 's' - 开始连续数据采集");
    ESP_LOGI(TAG, "  5. 'q' - 退出程序");
    ESP_LOGI(TAG, "=========================================");

    // 9. 启动主任务
    xTaskCreate(main_task, "main_task", 8192, NULL, 5, NULL);
}


/* ======================== 函数定义 ======================== */

/**
 * @brief 主任务，处理用户输入和传感器数据
 */
static void main_task(void *pvParameters)
{
    int input;
    bool running = true;

    while (running) {
        // 检查串口输入
        input = get_char_nonblocking();
        if (input != -1) {
            switch (input) {
                case 'h':
                case 'H':
                    mpu6500_show_help();
                    break;
                    
                case 'c':
                case 'C':
                    calibrate_sensors();
                    break;
                    
                case 't':
                case 'T':
                    verify_sensor_functionality();
                    break;
                    
                case 's':
                case 'S':
                    ESP_LOGI(TAG, "开始连续数据采集，按任意键停止...");
                    run_continuous_data_acquisition();
                    break;
                    
                case 'q':
                case 'Q':
                    ESP_LOGI(TAG, "正在退出程序...");
                    running = false;
                    break;
                    
                default:
                    // 处理其他命令（如输出模式切换）
                    bool reset_filter = mpu6500_process_serial_command(input, &g_output_mode);
                    if (reset_filter) {
                        attitude_filter_reset();
                        ESP_LOGI(TAG, "滤波器已重置");
                    }
                    break;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // 清理资源
    ESP_LOGI(TAG, "清理资源...");
    mpu6500_deinit(&g_mpu);
    ESP_LOGI(TAG, "程序退出");
    ESP_LOGI(TAG, "Press RST to reset");
    vTaskDelete(NULL);
}

/**
 * @brief 执行传感器校准流程
 */
static void calibrate_sensors(void)
{
    ESP_LOGI(TAG, "开始传感器校准流程...");
    ESP_LOGI(TAG, "请注意: 校准期间请保持传感器静止！");
    
    // 1. 加速度计校准
    ESP_LOGI(TAG, "步骤1: 加速度计校准");
    ESP_LOGI(TAG, "请将设备水平放置(Z轴朝上)");
    wait_for_any_key();
    
    esp_err_t ret = mpu6500_calibrate_accel(&g_mpu, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "加速度计校准失败: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "加速度计校准成功");
    }
    
    // 2. 陀螺仪校准
    ESP_LOGI(TAG, "\n步骤2: 陀螺仪校准");
    ESP_LOGI(TAG, "请将设备保持静止");
    wait_for_any_key();
    
    ret = mpu6500_calibrate_gyro(&g_mpu, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "陀螺仪校准失败: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "陀螺仪校准成功");
    }
    
    ESP_LOGI(TAG, "传感器校准完成!");
    ESP_LOGI(TAG, "按'h'查看帮助，按't'测试校准效果");
}

/**
 * @brief 验证传感器基本功能
 */
static void verify_sensor_functionality(void)
{
    ESP_LOGI(TAG, "开始传感器功能验证...");
    
    // 1. 读取WHO_AM_I寄存器
    uint8_t whoami;
    esp_err_t ret = mpu6500_read_whoami(&g_mpu, &whoami);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "WHO_AM_I寄存器: 0x%02X", whoami);
        if (whoami == 0x70) {
            ESP_LOGI(TAG, "✓ 传感器识别正确 (MPU6500)");
        } else {
            ESP_LOGW(TAG, "⚠ 传感器ID异常");
        }
    }
    
    // 2. 读取温度
    int16_t temp_raw;
    ret = mpu6500_read_temp(&g_mpu, &temp_raw);
    if (ret == ESP_OK) {
        float temp_c = (temp_raw / 333.87f) + 21.0f; // MPU6500温度转换公式
        ESP_LOGI(TAG, "温度传感器: %.2f°C", temp_c);
        ESP_LOGI(TAG, "✓ 温度传感器工作正常");
    }
    
    // 3. 单次读取加速度计和陀螺仪
    float accel_g[3], gyro_dps[3];
    ret = mpu6500_read_sensor_data(&g_mpu, NULL, accel_g, gyro_dps);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "单次读取测试:");
        print_sensor_data(accel_g, gyro_dps);
        ESP_LOGI(TAG, "✓ 数据读取正常");
    }
    
    // 4. 检查校准状态
    float gyro_bias[3], accel_bias[6];
    esp_err_t has_calibration = mpu6500_get_calibration(gyro_bias, accel_bias);
    if (has_calibration == ESP_OK) {
        ESP_LOGI(TAG, "✓ 已加载校准数据");
        ESP_LOGI(TAG, "  陀螺仪零偏: [%.3f, %.3f, %.3f] °/s", 
                gyro_bias[0], gyro_bias[1], gyro_bias[2]);
        ESP_LOGI(TAG, "  加速度计零偏: [%.4f, %.4f, %.4f] g", 
                accel_bias[0], accel_bias[1], accel_bias[2]);
    } else {
        ESP_LOGW(TAG, "⚠ 未找到校准数据，建议运行校准 ('c')");
    }
    
    // 5. 测试姿态滤波器
    attitude_data_t attitude;
    attitude_filter_update(accel_g, gyro_dps, 1/SAMPLE_RATE_HZ, &attitude); // 4ms间隔
    ESP_LOGI(TAG, "姿态滤波器测试:");
    print_attitude_data(&attitude);
    ESP_LOGI(TAG, "✓ 姿态滤波器工作正常");
    
    ESP_LOGI(TAG, "功能验证完成!");
}

/**
 * @brief 运行连续数据采集
 */
static void run_continuous_data_acquisition(void)
{
    ESP_LOGI(TAG, "开始连续数据采集 (按任意键停止)...");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t sample_interval = pdMS_TO_TICKS(1000 / SAMPLE_RATE_HZ);
    uint32_t sample_count = 0;
    bool running = true;
    
    while (running) {
        // 检查是否有按键输入
        if (get_char_nonblocking() != -1) {
            running = false;
            break;
        }
        
        // 读取传感器数据
        float accel_g[3], gyro_dps[3];
        esp_err_t ret = mpu6500_read_sensor_data(&g_mpu, NULL, accel_g, gyro_dps);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "读取传感器数据失败");
            break;
        }
        
        // 更新姿态滤波器
        attitude_data_t attitude;
        attitude_filter_update(accel_g, gyro_dps, 1.0f / SAMPLE_RATE_HZ, &attitude);
        
        // 根据输出模式显示数据
        switch (g_output_mode) {
            case OUTPUT_MODE_DETAILED:
                printf("[%8lu] ", sample_count++);
                print_attitude_data(&attitude);
                break;
                
            case OUTPUT_MODE_SIMPLE:
                printf("%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\n",
                       accel_g[0], accel_g[1], accel_g[2],
                       gyro_dps[0], gyro_dps[1], gyro_dps[2]);
                break;
                
            case OUTPUT_MODE_JSON:
                printf("{\"ts\":%lu,\"accel\":[%.3f,%.3f,%.3f],\"gyro\":[%.2f,%.2f,%.2f],\"euler\":[%.2f,%.2f,%.2f]}\n",
                       attitude.timestamp_us,
                       accel_g[0], accel_g[1], accel_g[2],
                       gyro_dps[0], gyro_dps[1], gyro_dps[2],
                       attitude.roll, attitude.pitch, attitude.yaw);
                break;
                
            case OUTPUT_MODE_QUATERNION:
                printf("%.4f,%.4f,%.4f,%.4f\n",
                       attitude.q0, attitude.q1, attitude.q2, attitude.q3);
                break;
        }
        
        // 精确延时控制采样率
        vTaskDelayUntil(&last_wake_time, sample_interval);
    }
    
    // 清空输入缓冲区
    uint8_t dummy;
    while (uart_read_bytes(UART_NUM, &dummy, 1, 0) > 0) {}
    
    ESP_LOGI(TAG, "连续数据采集已停止");
}

/**
 * @brief 打印传感器数据
 */
static void print_sensor_data(const float accel[3], const float gyro[3])
{
    printf("加速度: X= %7.3f g, Y= %7.3f g, Z= %7.3f g\n", 
           accel[0], accel[1], accel[2]);
    printf("角速度: X=%8.2f °/s, Y=%8.2f °/s, Z=%8.2f °/s\n",
           gyro[0], gyro[1], gyro[2]);
}

/**
 * @brief 打印姿态数据
 */
static void print_attitude_data(const attitude_data_t *attitude)
{
    printf("欧拉角: Roll=%6.2f°, Pitch=%6.2f°, Yaw=%6.2f° | ",
           attitude->roll, attitude->pitch, attitude->yaw);
    printf("四元数: [%.3f, %.3f, %.3f, %.3f] | ",
           attitude->q0, attitude->q1, attitude->q2, attitude->q3);
    printf("角速度: [%6.1f, %6.1f, %6.1f] °/s\n",
           attitude->gyro_x, attitude->gyro_y, attitude->gyro_z);
}

/**
 * @brief 处理串口命令
 * @param ch 输入字符
 */
bool mpu6500_process_serial_command(int ch, output_mode_t *output_mode)
{
    bool reset_filter = false;

    if (ch == EOF)
    {
        return reset_filter;
    }

    switch (ch)
    {
    case 'd':
    case 'D':
        *output_mode = OUTPUT_MODE_DETAILED;
        printf("\n切换到详细输出模式\n");
        break;
    case 's':
    case 'S':
        *output_mode = OUTPUT_MODE_SIMPLE;
        printf("\n切换到简化输出模式(适合绘图)\n");
        break;
    case 'j':
    case 'J':
        *output_mode = OUTPUT_MODE_JSON;
        printf("\n切换到JSON输出模式\n");
        break;
    case 'q':
    case 'Q':
        *output_mode = OUTPUT_MODE_QUATERNION;
        printf("\n切换到四元数输出模式\n");
        break;
    case 'r':
    case 'R':
        printf("\n重置滤波器...\n");
        reset_filter = true;
        break;
    case 'c':
    case 'C':
        printf("\n清除NVS校准数据...\n");
        attitude_filter_clear_nvs_calibration();
        break;
    case 'h':
    case 'H':
        mpu6500_show_help();
        break;
    }

    return reset_filter;
}

/**
 * @brief 打印帮助信息
 */
void mpu6500_show_help(void)
{   
    ESP_LOGI(TAG, "=========================================");
    ESP_LOGI(TAG, "按以下按键进行交互:");
    ESP_LOGI(TAG, "  1. 'h' - 显示此界面信息");
    ESP_LOGI(TAG, "  2. 'c' - 执行传感器校准");
    ESP_LOGI(TAG, "  3. 't' - 测试基本功能");
    ESP_LOGI(TAG, "  4. 's' - 开始连续数据采集");
    ESP_LOGI(TAG, "  5. 'q' - 退出程序");
    ESP_LOGI(TAG, "=========================================");
}

/**
 * @brief 初始化UART用于用户输入
 */
static void init_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART初始化完成");
}

/**
 * @brief 非阻塞获取一个字符
 * @return 读取到的字符，如果没有字符则返回-1
 */
static int get_char_nonblocking(void)
{
    uint8_t data;
    int length = uart_read_bytes(UART_NUM, &data, 1, 0);
    if (length > 0) {
        return data;
    }
    return -1;
}

/**
 * @brief 等待任意按键
 */
static void wait_for_any_key(void)
{
    ESP_LOGI(TAG, "按任意键继续...");
    while (get_char_nonblocking() == -1) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // 清空输入缓冲区
    uint8_t dummy;
    while (uart_read_bytes(UART_NUM, &dummy, 1, 0) > 0) {}
}
