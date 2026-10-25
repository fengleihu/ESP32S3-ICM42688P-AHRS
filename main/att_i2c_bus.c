#include "att_i2c_bus.h"

#include <stdio.h>

#include "esp_log.h"
#include "sdkconfig.h"

static const char *TAG = "i2c-bus";

#define PIN_SDA CONFIG_APP_OLED_I2C_SDA_GPIO
#define PIN_SCL CONFIG_APP_OLED_I2C_SCL_GPIO
#define I2C_HZ  CONFIG_APP_OLED_I2C_HZ

static i2c_master_bus_handle_t s_bus;
static bool s_inited;

esp_err_t app_i2c_bus_init(void)
{
    if (s_inited) {
        return ESP_OK;
    }
    if (PIN_SDA < 0 || PIN_SCL < 0) {
        ESP_LOGE(TAG, "I2C 引脚未配置（SDA=%d SCL=%d）", PIN_SDA, PIN_SCL);
        return ESP_ERR_INVALID_ARG;
    }

    i2c_master_bus_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C 总线初始化失败(SDA=%d SCL=%d): %s", PIN_SDA, PIN_SCL,
                 esp_err_to_name(err));
        return err;
    }
    s_inited = true;
    ESP_LOGI(TAG, "I2C 总线就绪：SDA=%d SCL=%d %dkHz", PIN_SDA, PIN_SCL, I2C_HZ / 1000);
    return ESP_OK;
}

i2c_master_bus_handle_t app_i2c_bus_get(void)
{
    return s_inited ? s_bus : NULL;
}

esp_err_t app_i2c_bus_recover(void)
{
    if (!s_inited || s_bus == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = i2c_master_bus_reset(s_bus);
    if (err == ESP_OK) {
        ESP_LOGW(TAG, "I2C 总线已复位（从总线锁死中恢复）");
    } else {
        ESP_LOGE(TAG, "I2C 总线复位失败：%s —— SDA/SCL 可能被拉低或断了，"
                      "检查接线与 3V3 供电", esp_err_to_name(err));
    }
    return err;
}

int app_i2c_scan(void)
{
    if (!s_inited) {
        return 0;
    }
    char line[160];
    int n = 0, count = 0;
    n += snprintf(line + n, sizeof(line) - (size_t)n, "I2C 扫描 (SDA=%d SCL=%d):", PIN_SDA, PIN_SCL);
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2c_master_probe(s_bus, a, 50) == ESP_OK) {
            count++;
            n += snprintf(line + n, sizeof(line) - (size_t)n, " 0x%02X", a);
            if (n > (int)sizeof(line) - 8) {
                break;
            }
        }
    }
    if (count == 0) {
        n += snprintf(line + n, sizeof(line) - (size_t)n, " （无器件应答）");
    }
    ESP_LOGI(TAG, "%s", line);
    return count;
}
