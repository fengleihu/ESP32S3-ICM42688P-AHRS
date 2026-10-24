/*
 * ssd1306.c — 7 针 SPI OLED（SSD1306 / SH1106）底层驱动
 *
 * 接线（与工程默认一致，可在 menuconfig 改）：
 *   D0/SCLK -> GPIO18   D1/MOSI -> GPIO17   RES -> GPIO16   DC -> GPIO15   CS -> GPIO14
 *   VCC 3V3  GND
 *
 * 只负责：SPI 初始化、SSD1306 初始化时序、把显存整体刷到屏上。
 * 画面内容由 oled_ui.c 渲染（纯 C，可在 PC 上验证）。
 *
 * 4 针 I2C 屏走同目录的 ssd1306_i2c.c；上层只调用 ssd1306_flush，
 * 由 ssd1306_dispatch.c 按 menuconfig 的选择转发。
 */
#include "ssd1306.h"
#include "ssd1306_backends.h"

#include <string.h>

#include "sdkconfig.h"

#if CONFIG_APP_OLED_BUS_SPI

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "oled_ui.h"

static const char *TAG = "ssd1306";

#define PIN_MOSI CONFIG_APP_OLED_MOSI_GPIO
#define PIN_CLK  CONFIG_APP_OLED_CLK_GPIO
#define PIN_CS   CONFIG_APP_OLED_CS_GPIO
#define PIN_DC   CONFIG_APP_OLED_DC_GPIO
#define PIN_RST  CONFIG_APP_OLED_RST_GPIO

#define SPI_HOST_USED SPI2_HOST

static spi_device_handle_t s_dev;
static bool s_ready;

static void dc_level(int level)
{
    gpio_set_level(PIN_DC, level);
}

/* 发送命令序列（DC=0） */
static esp_err_t send_cmds(const uint8_t *cmds, size_t n)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    spi_transaction_t t = {
        .length = n * 8,
        .tx_buffer = cmds,
    };
    dc_level(0);
#if CONFIG_APP_OLED_DC_SETUP_US > 0
    /* DC 线较长/带电容时，切换后给一点建立时间，避免有字节被当成命令 */
    esp_rom_delay_us(CONFIG_APP_OLED_DC_SETUP_US);
#endif
    esp_err_t err = spi_device_polling_transmit(s_dev, &t);
    dc_level(1);
    return err;
}

static esp_err_t send_cmd(uint8_t cmd)
{
    return send_cmds(&cmd, 1);
}

/* SSD1306 上电初始化序列 */
static esp_err_t panel_init_sequence(void)
{
    static const uint8_t init[] = {
        0xAE,             /* 关显示 */
        0xD5, 0x80,       /* 时钟分频/振荡频率 */
#if OLED_UI_H >= 64
        0xA8, 0x3F,       /* 多路复用比 64（0.96 寸） */
#else
        0xA8, 0x1F,       /* 多路复用比 32（0.91 寸） */
#endif
        0xD3, 0x00,       /* 显示偏移 0 */
        0x40,             /* 起始行 0 */
#if CONFIG_APP_OLED_SH1106
        0xAD, 0x8B,       /* SH1106 的内部 DC-DC 开（和 SSD1306 的 0x8D 不一样） */
#else
        0x8D, 0x14,       /* SSD1306 电荷泵开 */
#endif
        0x20, 0x00,       /* 内存寻址模式：水平（SH1106 不支持，会被忽略，无妨） */
        0xA1,             /* 段重映射（左右方向） */
        0xC8,             /* COM 扫描方向（上下方向） */
#if OLED_UI_H >= 64
        0xDA, 0x12,       /* COM 引脚配置（0.96 寸） */
#else
        0xDA, 0x02,       /* COM 引脚配置（0.91 寸 128x32 用顺序方式） */
#endif
        0x81, CONFIG_APP_OLED_CONTRAST, /* 对比度 */
        0xD9, 0xF1,       /* 预充电周期 */
        0xDB, 0x40,       /* VCOMH 取消选择电平 */
        0xA4,             /* 显示跟随显存 */
        0xA6,             /* 正常显示（非反色） */
        0x2E,             /* 关闭滚动 */
        0xAF,             /* 开显示 */
    };
    esp_err_t err = send_cmds(init, sizeof(init));
    if (err != ESP_OK) {
        return err;
    }

#if CONFIG_APP_OLED_MIRROR_X
    send_cmd(0xA0); /* 段重映射反过来：左右镜像 */
#endif
#if CONFIG_APP_OLED_MIRROR_Y
    send_cmd(0xC0); /* COM 扫描方向反过来：上下镜像 */
#endif
    return ESP_OK;
}

esp_err_t ssd1306_spi_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    /* RST 复位脉冲 */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_RST) | (1ULL << PIN_DC),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(PIN_DC, 1);
    gpio_set_level(PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = -1,
        .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = OLED_UI_FB_SIZE + 16,
    };
    esp_err_t err = spi_bus_initialize(SPI_HOST_USED, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI 总线初始化失败: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t dev = {
        .clock_speed_hz = CONFIG_APP_OLED_SPI_HZ,
        .mode = CONFIG_APP_OLED_SPI_MODE, /* 标准 0；画面错乱时可试 3 */
        .spics_io_num = PIN_CS,
        .queue_size = 1,
    };
    err = spi_bus_add_device(SPI_HOST_USED, &dev, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "添加 SPI 设备失败: %s", esp_err_to_name(err));
        return err;
    }
    s_ready = true;

    err = panel_init_sequence();
    if (err != ESP_OK) {
        s_ready = false;
        return err;
    }

    /* 先清屏，避免上电花屏 */
    static uint8_t blank[OLED_UI_FB_SIZE];
    memset(blank, 0, sizeof(blank));
    ssd1306_spi_flush(blank);

    ESP_LOGI(TAG, "OLED 就绪：MOSI=%d CLK=%d CS=%d DC=%d RST=%d %dkHz mode=%d 尺寸=%dx%d",
             PIN_MOSI, PIN_CLK, PIN_CS, PIN_DC, PIN_RST, CONFIG_APP_OLED_SPI_HZ / 1000,
             CONFIG_APP_OLED_SPI_MODE, OLED_UI_W, OLED_UI_H);
    return ESP_OK;
}

esp_err_t ssd1306_spi_flush(const uint8_t *fb)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

#if CONFIG_APP_OLED_SH1106
    /* SH1106：只能按页写，且显存列从 2 开始 */
    for (int page = 0; page < OLED_UI_PAGES; page++) {
        uint8_t cmds[3] = {(uint8_t)(0xB0 | page), 0x02, 0x10}; /* 列低/高 4 位 = 2 */
        esp_err_t err = send_cmds(cmds, sizeof(cmds));
        if (err != ESP_OK) {
            return err;
        }
        spi_transaction_t t = {
            .length = OLED_UI_W * 8,
            .tx_buffer = &fb[page * OLED_UI_W],
        };
        dc_level(1);
        err = spi_device_polling_transmit(s_dev, &t);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
#else
    /* SSD1306：设一次窗口，然后整帧一次写 */
    uint8_t window[] = {
        0x21, 0x00, 0x7F,                       /* 列 0..127 */
        0x22, 0x00, (uint8_t)(OLED_UI_PAGES - 1), /* 页 0..PAGES-1 */
    };
    esp_err_t err = send_cmds(window, sizeof(window));
    if (err != ESP_OK) {
        return err;
    }

    spi_transaction_t t = {
        .length = OLED_UI_FB_SIZE * 8,
        .tx_buffer = fb,
    };
    dc_level(1);
    return spi_device_polling_transmit(s_dev, &t);
#endif
}

esp_err_t ssd1306_spi_display_on(bool on)
{
    return send_cmd(on ? 0xAF : 0xAE);
}

esp_err_t ssd1306_spi_set_contrast(uint8_t contrast)
{
    uint8_t cmds[2] = {0x81, contrast};
    return send_cmds(cmds, sizeof(cmds));
}

#endif /* CONFIG_APP_OLED_BUS_SPI */
