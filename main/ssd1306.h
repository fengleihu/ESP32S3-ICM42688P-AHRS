/*
 * ssd1306.h — 0.96" 7P SPI OLED（SSD1306 / SH1106，128x64）底层驱动
 *
 * 接线（与工程默认一致，可在 menuconfig 改）：
 *   D0/SCLK -> GPIO18   D1/MOSI -> GPIO17   RES -> GPIO16   DC -> GPIO15   CS -> GPIO14
 *   VCC 3V3  GND
 *
 * 只负责：SPI 初始化、SSD1306 初始化时序、把 1024 字节显存整体刷到屏上。
 * 画面内容由 oled_ui.c 渲染（纯 C，可在 PC 上验证）。
 */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 初始化 SPI 总线与屏幕（幂等；失败返回错误，调用方可继续运行） */
esp_err_t ssd1306_init(void);

/** 整帧刷新：fb 为 128*64/8 = 1024 字节，格式与 SSD1306 显存一致 */
esp_err_t ssd1306_flush(const uint8_t *fb);

/** 开关显示（0xAE/0xAF），省电用 */
esp_err_t ssd1306_display_on(bool on);

/** 设置对比度 0~255 */
esp_err_t ssd1306_set_contrast(uint8_t contrast);

#ifdef __cplusplus
}
#endif
