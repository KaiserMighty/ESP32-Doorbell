#include <stdio.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include "ssd1306.h"

#define MPU6050_ADDR        0x68
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   19
#define I2C_MASTER_SCL_IO   18
#define I2C_MASTER_FREQ_HZ  100000

float sensitivity = 0.2f;

static void i2c_master_init()
{
    i2c_config_t conf =
    {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static void mpu6050_init(void)
{
    uint8_t data[2];
    data[0] = 0x6B;  // PWR_MGMT_1 register
    data[1] = 0x00;  // Wake up device
    esp_err_t ret = i2c_master_write_to_device(
        I2C_MASTER_NUM, MPU6050_ADDR, data, 2, pdMS_TO_TICKS(100)
    );
    if (ret != ESP_OK) {
        ESP_LOGE("MPU6050", "Failed to wake up MPU6050");
    } else {
        ESP_LOGI("MPU6050", "MPU6050 initialized");
    }
}

int16_t read_axis(uint8_t reg)
{
    uint8_t data[2];
    esp_err_t ret = i2c_master_write_read_device(
        I2C_MASTER_NUM,
        MPU6050_ADDR,
        &reg, 1,
        data, 2,
        pdMS_TO_TICKS(1000)
    );
    if (ret != ESP_OK)
    {
        ESP_LOGE("MPU6050", "Failed to read axis reg 0x%02X", reg);
        return 0;
    }
    return (int16_t)((data[0] << 8) | data[1]);
}

void app_main(void)
{
    i2c_master_init();
    mpu6050_init();
    ssd1306_init(I2C_MASTER_NUM);
    ssd1306_clear(I2C_MASTER_NUM);

    while (1)
    {
        int16_t ax = read_axis(0x3B);
        int16_t ay = read_axis(0x3D);
        int16_t az = read_axis(0x3F);
        float a = sqrtf(ax * ax + ay * ay + az * az) / 16384.0f;

        if (fabs(a - 1.0f) > sensitivity)
        {
            ESP_LOGI("MONITOR", "Vibration Detected! Raw: ax=%d ay=%d az=%d | Normalized: %.2f g", ax, ay, az, a);
            ssd1306_draw_text(I2C_MASTER_NUM, "VIBRATION DETECTED");
            vTaskDelay(pdMS_TO_TICKS(1000));
            ssd1306_clear(I2C_MASTER_NUM);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}