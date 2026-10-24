/*
 * ssd1306_dispatch.c — 按 menuconfig 的选择把公共 API 转发到 SPI 或 I2C 后端
 *
 * 上层（oled_test.c）只调用 ssd1306_init / ssd1306_flush，不需要知道总线类型。
 * 这样 SPI 屏和 I2C 屏可以共用同一套图案、同一套日志、同一套判定流程 ——
 * 换屏时只改一个 Kconfig 选项，其余判据完全可比。
 */
#include "sdkconfig.h"
#include "ssd1306.h"
#include "ssd1306_backends.h"

esp_err_t ssd1306_init(void)
{
#if CONFIG_APP_OLED_BUS_I2C
    return ssd1306_i2c_init();
#else
    return ssd1306_spi_init();
#endif
}

esp_err_t ssd1306_flush(const uint8_t *fb)
{
#if CONFIG_APP_OLED_BUS_I2C
    return ssd1306_i2c_flush(fb);
#else
    return ssd1306_spi_flush(fb);
#endif
}

esp_err_t ssd1306_display_on(bool on)
{
#if CONFIG_APP_OLED_BUS_I2C
    return ssd1306_i2c_display_on(on);
#else
    return ssd1306_spi_display_on(on);
#endif
}

esp_err_t ssd1306_set_contrast(uint8_t contrast)
{
#if CONFIG_APP_OLED_BUS_I2C
    return ssd1306_i2c_set_contrast(contrast);
#else
    return ssd1306_spi_set_contrast(contrast);
#endif
}
