/*
 * icm42688p.h — ICM-42688-P 六轴 IMU 驱动（I2C，ESP-IDF v5.x 新版 i2c_master）
 *
 * 由本工程的 mpu6050.c 改写而来：对外接口保持一致（init / read /
 * calibrate_gyro / read_reg / write_reg），main.c 只需换类型和函数名。
 *
 * 量程固定为：加速度 ±2g（16384 LSB/g）、陀螺 ±500°/s（65.5 LSB/(°/s)），
 * 输出数据率 100 Hz，与旧工程一致，便于直接对比。
 *
 * 与 MPU6050 的差异（重要）：
 *   - WHO_AM_I 寄存器 0x75，正确值 0x47；
 *   - 数据寄存器从 0x1D 起连续 14 字节：温度(2) + 加速度 XYZ(6) + 陀螺 XYZ(6)；
 *   - 上电需要先软复位（0x11=0x01），再配置量程/ODR，最后 PWR_MGMT0=0x0F 进
 *     "低噪声"模式；陀螺从使能到数据有效需要约 45 ms；
 *   - I2C 地址由 AP_AD0 决定：接地 0x68、接 VDD 0x69。
 *
 * 安装朝向：本项目的 ICM42688P 模块 +X=前、+Y=左、+Z=上，正好与机体坐标系一致，
 * 所以 main.c 里的 imu_to_body() 是单位变换，不做任何旋转。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ICM42688P_ADDR_AD0_LOW  0x68
#define ICM42688P_ADDR_AD0_HIGH 0x69
#define ICM42688P_WHO_AM_I_VAL  0x47

/** 与旧工程一致：400kHz */
#define ICM42688P_I2C_SPEED_HZ 100000

typedef struct {
    float accel_g[3];  /* g       */
    float gyro_dps[3]; /* °/s，已扣除零偏 */
    float temp_c;      /* ℃       */
    int16_t raw[7];    /* ax ay az temp gx gy gz（原始值，调试用） */
} icm42688p_sample_t;

typedef struct {
    i2c_master_dev_handle_t dev;
    uint8_t addr;
    float accel_lsb_per_g;
    float gyro_lsb_per_dps;
    float gyro_bias_dps[3]; /* 开机静态校准得到的零偏 */
    /* 加速度计六面标定（偏移 + 标度）。未标定时 bias=0、scale=1。 */
    float accel_bias_g[3];
    float accel_scale[3];
    uint8_t who_am_i;
} icm42688p_t;

typedef struct {
    float bias_g[3];  /* 零 g 偏移 */
    float scale[3];   /* 标度修正（无量纲，1.0 = 不动） */
    bool valid;
} icm42688p_accel_cal_t;

/**
 * 探测并初始化传感器（软复位 → 配置量程/ODR → 进入低噪声模式）。
 * @param bus       已创建的 I2C 主机总线
 * @param addr      7 位地址（0x68 / 0x69）
 * @param sample_hz 目标采样率，只支持 100/200/1000 Hz 这几档，其它值按 100 Hz 处理
 * @param dlpf_cfg  保留参数（ICM42688P 的 UI 滤波器默认 ODR/2，与旧工程 DLPF=3 接近）
 */
esp_err_t icm42688p_init(icm42688p_t *imu, i2c_master_bus_handle_t bus, uint8_t addr,
                         uint32_t sample_hz, uint8_t dlpf_cfg);

/** 读取一帧数据（14 字节突发读，含温度） */
esp_err_t icm42688p_read(icm42688p_t *imu, icm42688p_sample_t *out);

/**
 * 静态零偏校准：采 samples 个样本求陀螺平均值作为零偏。期间必须保持静止。
 * 结果写入 imu->gyro_bias_dps，后续 read 自动扣除。
 */
esp_err_t icm42688p_calibrate_gyro(icm42688p_t *imu, int samples, int sample_delay_ms,
                                   float *out_std_dps);

/** 读单个寄存器 */
esp_err_t icm42688p_read_reg(icm42688p_t *imu, uint8_t reg, uint8_t *val);

/**
 * 设置六面加速度计标定：accel_g = (raw/lsb - bias_g) * scale，并立即生效、存 NVS。
 *
 * 为什么要做：未标定时实测水平读数 ax=+14 mg、ay=-14 mg、az=1010 mg —— 0.8° 的
 * 横滚/俯仰偏置加 1% 标度误差。倾斜补偿用加速度计定姿态，这个偏置会被
 * tan(磁倾角) 放大成航向误差（dip≈54° 时 1.6° 倾角误差 ≈ 2.2° 航向误差）。
 *
 * 合理性检查：|bias| ≤ 0.15 g、0.85 ≤ scale ≤ 1.2，超出直接拒绝（说明六面数据不对）。
 */
esp_err_t icm42688p_accel_cal_set(icm42688p_t *imu,
                                  const icm42688p_accel_cal_t *cal);

/** 取当前生效的加速度计标定 */
void icm42688p_accel_cal_get(const icm42688p_t *imu, icm42688p_accel_cal_t *out);

/** 从 NVS 载入（没有就用单位标定）。init 之后调用。 */
esp_err_t icm42688p_accel_cal_load(icm42688p_t *imu);

/** 清除 NVS 里的标定并恢复单位标定 */
esp_err_t icm42688p_accel_cal_clear(icm42688p_t *imu);

/** 写单个寄存器 */
esp_err_t icm42688p_write_reg(icm42688p_t *imu, uint8_t reg, uint8_t val);

#ifdef __cplusplus
}
#endif
