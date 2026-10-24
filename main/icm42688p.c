/*
 * icm42688p.c — ICM-42688-P 驱动实现（I2C）
 *
 * 寄存器与初始化流程依据 TDK InvenSense ICM-42688-P 数据手册（Bank 0）：
 *   0x11 DEVICE_CONFIG    bit0 = 软复位
 *   0x1D TEMP_DATA1      温度高字节
 *   0x1F ACCEL_DATA_X1   加速度起始
 *   0x25 GYRO_DATA_X1    陀螺起始
 *   0x4E PWR_MGMT0       bit3:2 = GYRO_MODE, bit1:0 = ACCEL_MODE（11 = 低噪声）
 *   0x4F GYRO_CONFIG0    bit7:5 = FS_SEL, bit3:0 = ODR
 *   0x50 ACCEL_CONFIG0   bit7:5 = AFS_SEL, bit3:0 = ODR
 *   0x75 WHO_AM_I        应为 0x47
 *
 * 注意：0x1D 起连续 14 字节正好是"温度 + 加速度 + 陀螺"，所以一次突发读完，
 * 与旧 MPU6050 的 0x3B 起 14 字节读法等价。
 */
#include "icm42688p.h"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "icm42688p";

/* ---------------- 寄存器 ---------------- */
#define REG_DEVICE_CONFIG   0x11
#define REG_TEMP_DATA1      0x1D
#define REG_ACCEL_DATA_X1   0x1F
#define REG_GYRO_DATA_X1    0x25
#define REG_INT_STATUS      0x2D
#define REG_PWR_MGMT0       0x4E
#define REG_GYRO_CONFIG0    0x4F
#define REG_ACCEL_CONFIG0   0x50
#define REG_WHO_AM_I        0x75

#define DEVICE_CONFIG_SOFT_RESET 0x01

/* 数据寄存器起始与长度：温度(2) + 加速度(6) + 陀螺(6) */
#define REG_DATA_START REG_TEMP_DATA1
#define DATA_LEN       14

/* ---------------- 量程 ---------------- */
#define GYRO_FS_SEL 2 /* 0:±2000 1:±1000 2:±500 3:±250 °/s */
#define ACCEL_AFS_SEL 3 /* 0:±16g 1:±8g 2:±4g 3:±2g */

#if GYRO_FS_SEL == 0
#define GYRO_LSB_PER_DPS 16.4f
#elif GYRO_FS_SEL == 1
#define GYRO_LSB_PER_DPS 32.8f
#elif GYRO_FS_SEL == 2
#define GYRO_LSB_PER_DPS 65.5f
#else
#define GYRO_LSB_PER_DPS 131.0f
#endif

#if ACCEL_AFS_SEL == 0
#define ACCEL_LSB_PER_G 2048.0f
#elif ACCEL_AFS_SEL == 1
#define ACCEL_LSB_PER_G 4096.0f
#elif ACCEL_AFS_SEL == 2
#define ACCEL_LSB_PER_G 8192.0f
#else
#define ACCEL_LSB_PER_G 16384.0f
#endif

/* 温度：T = raw / 132.48 + 25 */
#define TEMP_LSB_PER_C 132.48f

#define I2C_TIMEOUT_MS 100

esp_err_t icm42688p_read_reg(icm42688p_t *imu, uint8_t reg, uint8_t *val)
{
    /* 首选标准"重复起始"读法 */
    esp_err_t err = i2c_master_transmit_receive(imu->dev, &reg, 1, val, 1, I2C_TIMEOUT_MS);
    if (err == ESP_OK) {
        return ESP_OK;
    }
    /* 有些模块/电平转换对重复起始支持不好：退化成"写寄存器地址 + 单独读" */
    esp_err_t err2 = i2c_master_transmit(imu->dev, &reg, 1, I2C_TIMEOUT_MS);
    if (err2 != ESP_OK) {
        return err2;
    }
    return i2c_master_receive(imu->dev, val, 1, I2C_TIMEOUT_MS);
}

esp_err_t icm42688p_write_reg(icm42688p_t *imu, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(imu->dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

/* ODR 编码（GYRO_CONFIG0/ACCEL_CONFIG0 的 bit3:0） */
static uint8_t odr_code(uint32_t sample_hz)
{
    switch (sample_hz) {
    case 200:
        return 0x06; /* 200 Hz */
    case 400:
        return 0x04; /* 4 kHz（不常用） */
    case 1000:
        return 0x05; /* 1 kHz */
    case 50:
        return 0x08; /* 50 Hz */
    case 25:
        return 0x09; /* 25 Hz */
    default:
        return 0x07; /* 100 Hz */
    }
}

esp_err_t icm42688p_init(icm42688p_t *imu, i2c_master_bus_handle_t bus, uint8_t addr,
                         uint32_t sample_hz, uint8_t dlpf_cfg)
{
    (void)dlpf_cfg; /* ICM42688P 的 UI 滤波器默认 ODR/2，与旧工程 DLPF=3 接近 */

    memset(imu, 0, sizeof(*imu));
    imu->addr = addr;
    imu->accel_lsb_per_g = ACCEL_LSB_PER_G;
    imu->gyro_lsb_per_dps = GYRO_LSB_PER_DPS;

    esp_err_t err = i2c_master_probe(bus, addr, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C 地址 0x%02X 无应答 (%s)。检查 SDA/SCL 接线、上拉、供电 3V3。",
                 addr, esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = ICM42688P_I2C_SPEED_HZ,
    };
    err = i2c_master_bus_add_device(bus, &dev_cfg, &imu->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "添加 I2C 设备失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 软复位：DEVICE_CONFIG(0x11) bit0 置 1，数据手册要求等待 ≥1 ms */
    icm42688p_write_reg(imu, REG_DEVICE_CONFIG, DEVICE_CONFIG_SOFT_RESET);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 复位后轮询 WHO_AM_I，确认芯片回来了 */
    uint8_t who = 0;
    for (int i = 0; i < 10; i++) {
        if (icm42688p_read_reg(imu, REG_WHO_AM_I, &who) == ESP_OK && who != 0x00 && who != 0xFF) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    imu->who_am_i = who;
    if (who != ICM42688P_WHO_AM_I_VAL) {
        ESP_LOGW(TAG, "WHO_AM_I = 0x%02X（期望 0x47），可能是兼容型号，继续尝试", who);
    }

    /* 量程 + ODR：陀螺 ±500°/s、加速度 ±2g、100 Hz（默认） */
    uint8_t gyro_cfg = (uint8_t)((GYRO_FS_SEL << 5) | odr_code(sample_hz));
    uint8_t accel_cfg = (uint8_t)((ACCEL_AFS_SEL << 5) | odr_code(sample_hz));
    icm42688p_write_reg(imu, REG_GYRO_CONFIG0, gyro_cfg);
    icm42688p_write_reg(imu, REG_ACCEL_CONFIG0, accel_cfg);

    /* PWR_MGMT0：陀螺 + 加速度都进低噪声模式（bit3:2 = 11, bit1:0 = 11） */
    icm42688p_write_reg(imu, REG_PWR_MGMT0, 0x0F);
    /* 陀螺从使能到输出有效约 45 ms，这里留 60 ms 余量 */
    vTaskDelay(pdMS_TO_TICKS(60));

    /* 回读校验，日志里能直接看到是否写进去了 */
    uint8_t pwr = 0, gcfg = 0, acfg = 0;
    icm42688p_read_reg(imu, REG_PWR_MGMT0, &pwr);
    icm42688p_read_reg(imu, REG_GYRO_CONFIG0, &gcfg);
    icm42688p_read_reg(imu, REG_ACCEL_CONFIG0, &acfg);
    ESP_LOGI(TAG, "ICM42688P 就绪：addr=0x%02X WHO_AM_I=0x%02X PWR=0x%02X GYRO=0x%02X ACCEL=0x%02X",
             addr, who, pwr, gcfg, acfg);
    ESP_LOGI(TAG, "量程：加速度 ±%dg，陀螺 ±%d°/s，ODR=%u Hz",
             (int)(32768.0f / ACCEL_LSB_PER_G),
             (int)(32768.0f / GYRO_LSB_PER_DPS), (unsigned)sample_hz);
    return ESP_OK;
}

esp_err_t icm42688p_read(icm42688p_t *imu, icm42688p_sample_t *out)
{
        uint8_t reg = REG_DATA_START;
    uint8_t buf[DATA_LEN];
    /* 本模块不支持重复起始：先写寄存器地址，再用独立事务读数据 */
    esp_err_t err = i2c_master_transmit(imu->dev, &reg, 1, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_master_receive(imu->dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    /* 大端。buf: 温度 0..1、加速度 2..7、陀螺 8..13
     * raw[] 保持与旧 MPU6050 驱动一致的顺序：ax ay az temp gx gy gz */
    out->raw[0] = (int16_t)((buf[2] << 8) | buf[3]);
    out->raw[1] = (int16_t)((buf[4] << 8) | buf[5]);
    out->raw[2] = (int16_t)((buf[6] << 8) | buf[7]);
    out->raw[3] = (int16_t)((buf[0] << 8) | buf[1]);
    out->raw[4] = (int16_t)((buf[8] << 8) | buf[9]);
    out->raw[5] = (int16_t)((buf[10] << 8) | buf[11]);
    out->raw[6] = (int16_t)((buf[12] << 8) | buf[13]);

    out->accel_g[0] = out->raw[0] / imu->accel_lsb_per_g;
    out->accel_g[1] = out->raw[1] / imu->accel_lsb_per_g;
    out->accel_g[2] = out->raw[2] / imu->accel_lsb_per_g;

    out->gyro_dps[0] = out->raw[4] / imu->gyro_lsb_per_dps - imu->gyro_bias_dps[0];
    out->gyro_dps[1] = out->raw[5] / imu->gyro_lsb_per_dps - imu->gyro_bias_dps[1];
    out->gyro_dps[2] = out->raw[6] / imu->gyro_lsb_per_dps - imu->gyro_bias_dps[2];

    out->temp_c = out->raw[3] / TEMP_LSB_PER_C + 25.0f;
    return ESP_OK;
}

esp_err_t icm42688p_calibrate_gyro(icm42688p_t *imu, int samples, int sample_delay_ms,
                                   float *out_std_dps)
{
    if (samples <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 先稳定 100 ms，避开上电/滤波器收敛的瞬态样本（旧工程踩过这个坑） */
    vTaskDelay(pdMS_TO_TICKS(100));

    double sum[3] = {0, 0, 0};
    double sum_sq[3] = {0, 0, 0};
    int ok = 0;
    icm42688p_sample_t s;

    for (int i = 0; i < samples; i++) {
        if (icm42688p_read(imu, &s) == ESP_OK) {
            for (int k = 0; k < 3; k++) {
                sum[k] += s.gyro_dps[k];
                sum_sq[k] += (double)s.gyro_dps[k] * s.gyro_dps[k];
            }
            ok++;
        }
        if (sample_delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(sample_delay_ms));
        }
    }

    if (ok < samples / 4) {
        ESP_LOGE(TAG, "校准失败：只有 %d/%d 个样本读取成功", ok, samples);
        return ESP_FAIL;
    }

    float worst_std = 0.0f;
    for (int k = 0; k < 3; k++) {
        double mean = sum[k] / ok;
        double var = sum_sq[k] / ok - mean * mean;
        if (var < 0) {
            var = 0;
        }
        imu->gyro_bias_dps[k] = (float)mean;
        float sd = (float)sqrt(var);
        if (sd > worst_std) {
            worst_std = sd;
        }
    }

    ESP_LOGI(TAG, "陀螺零偏校准完成 (%d 样本)：bias = [%.3f, %.3f, %.3f] °/s，噪声 %.3f °/s",
             ok, imu->gyro_bias_dps[0], imu->gyro_bias_dps[1], imu->gyro_bias_dps[2],
             worst_std);
    if (worst_std > 0.5f) {
        ESP_LOGW(TAG, "校准期间抖动较大（σ=%.3f °/s），建议静止重来", worst_std);
    }
    if (out_std_dps) {
        *out_std_dps = worst_std;
    }
    return ESP_OK;
}
