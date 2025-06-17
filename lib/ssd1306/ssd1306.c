#include "ssd1306.h"
#include <string.h>
#include "freertos/task.h"

static const uint8_t init_sequence[] =
{
    0xAE,       // Display OFF
    0x20, 0x00, // Horizontal addressing
    0xB0,       // Page Start Address
    0xC8,       // COM scan direction remapped
    0x00,       // Low column address
    0x10,       // High column address
    0x40,       // Start line address
    0x81, 0xFF, // Contrast
    0xA1,       // Segment remap
    0xA6,       // Normal display
    0xA8, 0x1F, // Multiplex ratio
    0xD3, 0x00, // Display offset
    0xD5, 0xF0, // Display clock divide
    0xD9, 0x22, // Pre-charge
    0xDA, 0x02, // COM pins config
    0xDB, 0x20, // VCOMH
    0x8D, 0x14, // Charge pump
    0xAF        // Display ON
};

static esp_err_t send_command(i2c_port_t i2c_num, uint8_t cmd)
{
    uint8_t buffer[2] = {0x00, cmd};
    return i2c_master_write_to_device(i2c_num, SSD1306_I2C_ADDR, buffer, 2, 1000 / portTICK_PERIOD_MS);
}

static esp_err_t send_data(i2c_port_t i2c_num, const uint8_t *data, size_t len)
{
    uint8_t *buf = malloc(len + 1);
    if (!buf) return ESP_ERR_NO_MEM;
    buf[0] = 0x40;
    memcpy(&buf[1], data, len);
    esp_err_t ret = i2c_master_write_to_device(i2c_num, SSD1306_I2C_ADDR, buf, len + 1, 1000 / portTICK_PERIOD_MS);
    free(buf);
    return ret;
}

esp_err_t ssd1306_init(i2c_port_t i2c_num)
{
    for (size_t i = 0; i < sizeof(init_sequence); ++i)
    {
        ESP_ERROR_CHECK(send_command(i2c_num, init_sequence[i]));
    }
    return ESP_OK;
}

void ssd1306_clear(i2c_port_t i2c_num)
{
    uint8_t zero_buffer[SSD1306_WIDTH] = {0};
    for (uint8_t page = 0; page < 4; ++page)
    {
        send_command(i2c_num, 0xB0 + page); // Set page address
        send_command(i2c_num, 0x00);        // Set lower column
        send_command(i2c_num, 0x10);        // Set higher column
        send_data(i2c_num, zero_buffer, SSD1306_WIDTH);
    }
}

esp_err_t ssd1306_draw_text(i2c_port_t i2c_num, const char *text)
{
    static const uint8_t font[27][5] =
    {
        [0] = {0x00,0x00,0x00,0x00,0x00},  // space
        [1] = {0x7C,0x12,0x11,0x12,0x7C},  // A
        [2] = {0x7F,0x49,0x49,0x49,0x36},  // B
        [3] = {0x3E,0x41,0x41,0x41,0x22},  // C
        [4] = {0x7F,0x41,0x41,0x22,0x1C},  // D
        [5] = {0x7F,0x49,0x49,0x49,0x41},  // E
        [6] = {0x7F,0x09,0x09,0x09,0x01},  // F
        [7] = {0x3E,0x41,0x49,0x49,0x7A},  // G
        [8] = {0x7F,0x08,0x08,0x08,0x7F},  // H
        [9] = {0x00,0x41,0x7F,0x41,0x00},  // I
        [10] = {0x20,0x40,0x41,0x3F,0x01}, // J
        [11] = {0x7F,0x08,0x14,0x22,0x41}, // K
        [12] = {0x7F,0x40,0x40,0x40,0x40}, // L
        [13] = {0x7F,0x02,0x0C,0x02,0x7F}, // M
        [14] = {0x7F,0x04,0x08,0x10,0x7F}, // N
        [15] = {0x3E,0x41,0x41,0x41,0x3E}, // O
        [16] = {0x7F,0x09,0x09,0x09,0x06}, // P
        [17] = {0x3E,0x41,0x51,0x21,0x5E}, // Q
        [18] = {0x7F,0x09,0x19,0x29,0x46}, // R
        [19] = {0x46,0x49,0x49,0x49,0x31}, // S
        [20] = {0x01,0x01,0x7F,0x01,0x01}, // T
        [21] = {0x3F,0x40,0x40,0x40,0x3F}, // U
        [22] = {0x1F,0x20,0x40,0x20,0x1F}, // V
        [23] = {0x3F,0x40,0x38,0x40,0x3F}, // W
        [24] = {0x63,0x14,0x08,0x14,0x63}, // X
        [25] = {0x07,0x08,0x70,0x08,0x07}, // Y
        [26] = {0x61,0x51,0x49,0x45,0x43}  // Z
    };

    ssd1306_clear(i2c_num);
    send_command(i2c_num, 0xB0); // Page 0
    send_command(i2c_num, 0x00); // Lower col
    send_command(i2c_num, 0x10); // Higher col

    while (*text)
    {
        char c = *text++;
        uint8_t index = 0;
        if (c == ' ') index = 0;
        else if (c >= 'A' && c <= 'Z') index = (c - 'A') + 1;
        else continue;

        send_data(i2c_num, font[index], 5);
        uint8_t space = 0x00;
        send_data(i2c_num, &space, 1);
    }

    return ESP_OK;
}
