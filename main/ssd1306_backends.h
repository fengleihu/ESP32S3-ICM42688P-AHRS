/*
 * ssd1306_backends.h — 两个后端（SPI / I2C）各自的原型
 *
 * ssd1306.c 里已经按 menuconfig 的选择把未选中的那个后端整段 #if 掉了，
 * 所以这里两个原型都声明不会有链接问题。ssd1306_dispatch.c 负责转发。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- SPI 后端（7 针：CLK/MOSI/RES/DC/CS）---- */
esp_err_t ssd1306_spi_init(void);
esp_err_t ssd1306_spi_flush(const uint8_t *fb);
esp_err_t ssd1306_spi_display_on(bool on);
esp_err_t ssd1306_spi_set_contrast(uint8_t contrast);

/* ---- I2C 后端（4 针：SCL/SDA）---- */
esp_err_t ssd1306_i2c_init(void);
esp_err_t ssd1306_i2c_flush(const uint8_t *fb);
esp_err_t ssd1306_i2c_display_on(bool on);
esp_err_t ssd1306_i2c_set_contrast(uint8_t contrast);

#ifdef __cplusplus
}
#endif
