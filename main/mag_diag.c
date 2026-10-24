/*
 * mag_diag.c — BMM350 现场诊断：把"到底在不在测量"这件事查清楚。
 *
 * 背景：接入后读数模长只有 0.x µT 且几乎不变（地球磁场是 25~65 µT），
 * 同时 DRDY 位一直是 0。这两件事合起来指向"芯片没在测量"，而不是"校准没转开"。
 *
 * 本诊断绕过补偿逻辑，直接看：
 *   1) PMU 状态寄存器：芯片到底在 suspend 还是 normal 模式
 *   2) 未补偿的 24 位原始计数值：是否在变
 *   3) 官方补偿后的值：和原始值是否一致
 *   4) 芯片 ID / 中断状态
 *
 * 三个数字一比就能定位：
 *   原始值在变、补偿后≈0  -> 补偿/OTP 问题（软件）
 *   原始值也不变          -> 芯片没测量（模式/时序问题）
 *   原始值和补偿后都在变，只是幅度小 -> 场地磁场真的弱，或校准数据有残留
 */
#include "mag_diag.h"

#include <math.h>
#include <stdio.h>

#include "bmm350.h"
#include "bmm350_common.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mag-diag";

void mag_diag_dump(struct bmm350_dev *dev)
{
    if (!dev) {
        return;
    }

    ESP_LOGI(TAG, "===== BMM350 现场诊断 =====");

    /* 1) 芯片 ID 与 PMU 状态 */
    uint8_t pmu = 0, int_status = 0;
    if (bmm350_get_regs(BMM350_REG_PMU_CMD_STATUS_0, &pmu, 1, dev) == BMM350_OK) {
        ESP_LOGI(TAG, "PMU 状态寄存器 = 0x%02X", pmu);
    } else {
        ESP_LOGE(TAG, "读 PMU 状态寄存器失败");
    }
    if (bmm350_get_interrupt_status(&int_status, dev) == BMM350_OK) {
        ESP_LOGI(TAG, "INT_STATUS = 0x%02X，DRDY = %d", int_status,
                 (int_status & BMM350_DRDY_DATA_REG_MSK) ? 1 : 0);
    }

    /* 2) 连续读 8 次未补偿原始值 + 官方补偿值，看它们在不在变 */
    ESP_LOGI(TAG, "序号 | 原始(raw_x, raw_y, raw_z) | 补偿后(µT)");
    int changed_raw = 0, changed_cal = 0;
    int32_t prev_raw[3] = {0, 0, 0};
    float prev_cal[3] = {0, 0, 0};

    for (int i = 0; i < 8; i++) {
        struct bmm350_raw_mag_data raw;
        struct bmm350_mag_temp_data cal;
        int8_t r1 = bmm350_read_uncomp_mag_temp_data(&raw, dev);
        int8_t r2 = bmm350_get_compensated_mag_xyz_temp_data(&cal, dev);
        if (r1 != BMM350_OK || r2 != BMM350_OK) {
            ESP_LOGE(TAG, "第 %d 次读取失败 raw=%d cal=%d", i, r1, r2);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        ESP_LOGI(TAG, "%2d | %8d %8d %8d | %8.2f %8.2f %8.2f", i,
                 (int)raw.raw_xdata, (int)raw.raw_ydata, (int)raw.raw_zdata,
                 cal.x, cal.y, cal.z);

        if (i > 0) {
            if (raw.raw_xdata != prev_raw[0] || raw.raw_ydata != prev_raw[1] ||
                raw.raw_zdata != prev_raw[2]) {
                changed_raw++;
            }
            if (fabsf(cal.x - prev_cal[0]) > 1e-3f || fabsf(cal.y - prev_cal[1]) > 1e-3f ||
                fabsf(cal.z - prev_cal[2]) > 1e-3f) {
                changed_cal++;
            }
        }
        prev_raw[0] = raw.raw_xdata;
        prev_raw[1] = raw.raw_ydata;
        prev_raw[2] = raw.raw_zdata;
        prev_cal[0] = cal.x;
        prev_cal[1] = cal.y;
        prev_cal[2] = cal.z;
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "原始值发生变化：%d/7 次；补偿值发生变化：%d/7 次", changed_raw, changed_cal);
    if (changed_raw == 0) {
        ESP_LOGE(TAG, "原始计数值完全不动 —— 芯片没有在测量。"
                      "检查：是否真的进入了 NORMAL_MODE、OTP 是否加载成功、供电是否够");
    } else if (changed_cal == 0) {
        ESP_LOGE(TAG, "原始值在变但补偿值不动 —— 补偿/OTP 数据有问题");
    } else if (fabsf(prev_cal[0]) < 1.0f && fabsf(prev_cal[1]) < 1.0f &&
               fabsf(prev_cal[2]) < 1.0f) {
        ESP_LOGE(TAG, "补偿值接近 0 —— 补偿系数或 OTP 数据异常");
    } else {
        ESP_LOGI(TAG, "测量通路看起来正常，磁场模长 %.1f µT",
                 sqrtf(prev_cal[0] * prev_cal[0] + prev_cal[1] * prev_cal[1] +
                       prev_cal[2] * prev_cal[2]));
    }
    ESP_LOGI(TAG, "===== 诊断结束 =====");
}
