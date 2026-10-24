/*
 * ssd1306_i2c.c — 4 针 I2C 小屏（SSD1306 / SSD1315，0.91 寸 128x32 或 0.96 寸 128x64）
 *
 * 为什么加这条路：7 针 SPI 屏有 CS/DC/RES 三根控制线，接错或时序不对的表现恰好是
 * "屏能亮、但画面完全乱"，排查面很大。I2C 只有 4 根线（VCC/GND/SCL/SDA），
 * 没有控制线，控制器的寻址窗口由命令字节自己带，天生少一整类故障。
 *
 * 和 SPI 后端共用同一个显存格式：128 列 x N 页，每页 128 字节，字节的 bit0 是该页最上一行。
 * 整帧数据一次性发给控制器（控制字节 0x40 = 后面都是数据）。
 */
#include "sdkconfig.h"

#if CONFIG_APP_OLED_BUS_I2C

#include "ssd1306.h"
#include "ssd1306_backends.h"
#include "att_i2c_bus.h"

#include <string.h>

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "oled_ui.h"

static const char *TAG = "ssd1306-i2c";

#define PIN_SDA CONFIG_APP_OLED_I2C_SDA_GPIO
#define PIN_SCL CONFIG_APP_OLED_I2C_SCL_GPIO
#define I2C_HZ  CONFIG_APP_OLED_I2C_HZ

/* SSD1306 的 I2C 地址只有两种可能；启动时两个都探，哪个应答用哪个 */
#define ADDR_PRIMARY   ((uint8_t)CONFIG_APP_OLED_I2C_ADDR)
#define ADDR_ALTERNATE ((uint8_t)(CONFIG_APP_OLED_I2C_ADDR == 0x3C ? 0x3D : 0x3C))

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static uint8_t s_addr;
static bool s_ready;

/* 命令缓冲：0x00 表示"后面都是命令字节" */
static esp_err_t send_cmds(const uint8_t *cmds, size_t n)
{
    uint8_t buf[64];
    if (n + 1 > sizeof(buf)) {
        return ESP_ERR_INVALID_SIZE;
    }
    buf[0] = 0x00;
    memcpy(&buf[1], cmds, n);
    return i2c_master_transmit(s_dev, buf, n + 1, 200);
}

static esp_err_t send_cmd(uint8_t cmd)
{
    return send_cmds(&cmd, 1);
}

/* 扫描搬到 i2c_bus.c 的 app_i2c_scan()，屏和传感器共用同一份实现 */

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
        0x8D, 0x14,       /* 电荷泵开 */
        0x20, 0x00,       /* 内存寻址模式：水平 */
        0xA1,             /* 段重映射 */
        0xC8,             /* COM 扫描方向 */
#if OLED_UI_H >= 64
        0xDA, 0x12,       /* COM 引脚配置：0.96 寸用 0x12（交替） */
#elif CONFIG_APP_OLED_COM_PINS_ALT
        0xDA, 0x12,       /* 128x32 但画面上下重复时改用它 */
#else
        0xDA, 0x02,       /* COM 引脚配置：0.91 寸用 0x02（顺序） */
#endif
        0x81, CONFIG_APP_OLED_CONTRAST, /* 对比度 */
        0xD9, 0xF1,       /* 预充电周期 */
        0xDB, 0x40,       /* VCOMH 取消选择电平 */
        0xA4,             /* 显示跟随显存 */
        0xA6,             /* 正常显示（非反色） */
        0x2E,             /* 关闭滚动 */
        0xAF,             /* 开显示 */
    };
    return send_cmds(init, sizeof(init));
}

esp_err_t ssd1306_i2c_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }
    if (PIN_SDA < 0 || PIN_SCL < 0) {
        ESP_LOGE(TAG, "I2C 引脚未配置（SDA=%d SCL=%d）", PIN_SDA, PIN_SCL);
        return ESP_ERR_INVALID_ARG;
    }

    /* 总线是共用的（传感器也挂在上面），谁先要谁负责建 */
    esp_err_t err = app_i2c_bus_init();
    if (err != ESP_OK) {
        return err;
    }
    s_bus = app_i2c_bus_get();

    /* 扫描 + 找地址：重试几次，方便"带电插屏"时不用重新上电 */
    s_addr = 0;
    for (int attempt = 0; attempt < 3 && s_addr == 0; attempt++) {
        if (attempt) {
            ESP_LOGW(TAG, "没找到屏，%d 秒后再试一次…", 2);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        app_i2c_scan();
        /* 两个可能地址都探，避免"地址不对就全黑"这种最难查的情况 */
        for (int i = 0; i < 2; i++) {
            uint8_t a = (i == 0) ? ADDR_PRIMARY : ADDR_ALTERNATE;
            if (i2c_master_probe(s_bus, a, 100) == ESP_OK) {
                s_addr = a;
                break;
            }
        }
    }
    if (s_addr == 0) {
        ESP_LOGE(TAG, "0x%02X 和 0x%02X 都不应答 —— 检查 VCC(3V3)/GND/SDA/SCL 接线，"
                      "以及模块排针是否焊好", ADDR_PRIMARY, ADDR_ALTERNATE);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "OLED 在 0x%02X 应答", s_addr);

    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = s_addr,
        .scl_speed_hz = I2C_HZ,
    };
    err = i2c_master_bus_add_device(s_bus, &dev, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "添加 I2C 设备失败: %s", esp_err_to_name(err));
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
    ssd1306_i2c_flush(blank);

    ESP_LOGI(TAG, "OLED 就绪：I2C SDA=%d SCL=%d %dkHz 地址=0x%02X 尺寸=%dx%d",
             PIN_SDA, PIN_SCL, I2C_HZ / 1000, s_addr, OLED_UI_W, OLED_UI_H);
    return ESP_OK;
}

esp_err_t ssd1306_i2c_flush(const uint8_t *fb)
{
    if (!s_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    /* 先把寻址窗口一次设好：列 0..127，页 0..PAGES-1（水平寻址模式） */
    uint8_t window[] = {0x21, 0x00, (uint8_t)(OLED_UI_W - 1),
                        0x22, 0x00, (uint8_t)(OLED_UI_PAGES - 1)};
    esp_err_t err = send_cmds(window, sizeof(window));
    if (err != ESP_OK) {
        return err;
    }

    /*
     * 然后只发数据，中途**不再插入任何命令**。
     *
     * 这里有个坑：水平寻址模式下控制器把每个字节都当像素数据吃进去，
     * 如果在数据流中间插 0xB0/页地址这类命令，它们会被当成像素数据写进显存，
     * 表现为画面里出现一条固定的乱码。所以窗口设一次，数据分块连着发完。
     *
     * 分块只是为了不依赖"单次 I2C 传输能有多长"，跟寻址无关；控制器自己
     * 会按窗口自动递增列/页。
     */
    uint8_t buf[1 + OLED_UI_W];
    buf[0] = 0x40; /* 控制字节：后面都是数据 */
    for (int page = 0; page < OLED_UI_PAGES; page++) {
        memcpy(&buf[1], &fb[page * OLED_UI_W], OLED_UI_W);
        err = i2c_master_transmit(s_dev, buf, sizeof(buf), 500);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK;
}

esp_err_t ssd1306_i2c_display_on(bool on)
{
    return send_cmd(on ? 0xAF : 0xAE);
}

esp_err_t ssd1306_i2c_set_contrast(uint8_t contrast)
{
    uint8_t cmds[2] = {0x81, contrast};
    return send_cmds(cmds, sizeof(cmds));
}

#endif /* CONFIG_APP_OLED_BUS_I2C */
