/*
 * att_i2c_bus.h — 共用的一条 I2C 总线
 *
 * 为什么单独抽出来：OLED 和传感器都要挂在这条总线上（SDA=1, SCL=2）。
 * 以前总线是在 ssd1306_i2c.c 里建的，那样"屏"和"传感器"就耦合在一起了 ——
 * 屏没接上会导致总线根本不建，传感器也就跟着测不了。
 * 现在谁先要谁负责建，另一个直接用。
 */
#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 建总线（幂等：已经建过就直接返回成功）。引脚取 menuconfig 里的 OLED I2C 配置。 */
esp_err_t app_i2c_bus_init(void);

/** 取总线句柄；未初始化返回 NULL */
i2c_master_bus_handle_t app_i2c_bus_get(void);

/**
 * 总线锁死后的恢复：发 9 个 SCL 时钟把被从机拉住的 SDA 释放，并复位驱动状态机。
 *
 * 为什么必须有：ESP-IDF 的 i2c 驱动在"清总线"失败后会停掉状态机，之后每次
 * 调用都直接返回 ESP_ERR_INVALID_STATE，不主动复位就永远恢复不了 —— 实测在
 * 板子被手拿着移动时（杜邦线抖动）出现过一次，之后 OLED、IMU、磁力计全部失效
 * 直到重启，整段测量作废。
 *
 * @return ESP_OK 恢复成功；ESP_ERR_INVALID_STATE 表示时钟拉不动 SDA
 *         （真的短路/断线/没供电，需要人工检查接线）
 */
esp_err_t app_i2c_bus_recover(void);

/** 扫描 0x08~0x77，把应答的地址打到日志，返回应答个数 */
int app_i2c_scan(void);

#ifdef __cplusplus
}
#endif
