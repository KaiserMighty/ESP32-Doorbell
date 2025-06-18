#include <stdio.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include "ssd1306.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>

#define MPU6050_ADDR        0x68
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   19
#define I2C_MASTER_SCL_IO   18
#define I2C_MASTER_FREQ_HZ  100000

#define WIFI_SSID       "SSID"
#define WIFI_PASS       "PASS"
#define BROADCAST_PORT  17388
#define BROADCAST_IP    "255.255.255.255"

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

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
        esp_wifi_connect();
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGW("NETWORK", "Disconnected from WiFi. Trying to reconnect...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI("NETWORK", "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

void wifi_init(void)
{
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_config =
    {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();
}

void send_udp_broadcast(const char *message)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
    {
        ESP_LOGI("NETWORK", "Failed to create socket: errno %d", errno);
        return;
    }

    int broadcastEnable = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0)
    {
        ESP_LOGE("NETWORK", "Failed to set broadcast option: errno %d", errno);
        close(sock);
        return;
    }

    struct sockaddr_in addr =
    {
        .sin_family = AF_INET,
        .sin_port = htons(BROADCAST_PORT),
        .sin_addr.s_addr = inet_addr(BROADCAST_IP),
    };

    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK)
    {
        ESP_LOGE("NETWORK", "ESP32 is not connected to Wi-Fi");
        return;
    }

    if (sendto(sock, message, strlen(message), 0, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        ESP_LOGE("NETWORK", "Error occurred during sending: errno %d", errno);

    close(sock);
}

void app_main(void)
{
    i2c_master_init();
    mpu6050_init();
    ssd1306_init(I2C_MASTER_NUM);
    ssd1306_clear(I2C_MASTER_NUM);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    wifi_init();

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
            send_udp_broadcast("DOORBELL_KNOCK");
            vTaskDelay(pdMS_TO_TICKS(1000));
            ssd1306_clear(I2C_MASTER_NUM);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}