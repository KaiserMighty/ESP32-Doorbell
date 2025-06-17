#pragma once

#include <stdint.h>
#include "driver/i2c.h"

#define SSD1306_I2C_ADDR     0x3C
#define SSD1306_WIDTH        128
#define SSD1306_HEIGHT       32
#define SSD1306_BUFFER_SIZE  (SSD1306_WIDTH * SSD1306_HEIGHT / 8)

esp_err_t ssd1306_init(i2c_port_t i2c_num);
void ssd1306_clear(i2c_port_t i2c_num);
esp_err_t ssd1306_draw_text(i2c_port_t i2c_num, const char *text);
