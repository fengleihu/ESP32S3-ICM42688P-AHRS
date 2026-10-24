/*
 * mag.h — BMM350 磁力计封装（基于 Bosch 官方组件 espressif/bmm350）
 *
 * 提供三件事：
 *   1) 初始化与读取（float 微特斯拉，Bosch 官方补偿已做）
 *   2) 磁校准（椭球拟合为主，min-max 兜底），转动设备采样，结果存 NVS 掉电不丢
 *   3) 轴向映射（Kconfig 可配，用于磁力计与陀螺仪丝印方向不一致时纠正）
 *
 * 校准方法为什么换过：第一版是 min-max（每轴取最大/最小值得偏移与缩放），
 * 它假设软磁只沿坐标轴、没有交叉轴耦合，而且少转半个圈就会把半径估小。
 * 实测这个模块用 min-max 校准后，"校准后磁场模长"随位置在 45~57 µT 之间变，
 * 说明模型不够。现在用椭球拟合（main/mag_fit.c，纯数学、有主机侧单元测试），
 * 同时解出硬磁偏移、三轴缩放和交叉轴耦合；样本覆盖率不足或拟合退化时
 * 自动回退到 min-max。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bosch 驱动句柄的前置声明：头文件不必把 bmm350.h 暴露给所有使用者 */
struct bmm350_dev;

typedef enum {
    MAG_CAL_METHOD_NONE = 0,
    MAG_CAL_METHOD_MINMAX = 1,    /* 旧方法，仅在椭球拟合失败时回退使用 */
    MAG_CAL_METHOD_ELLIPSOID = 2, /* 默认 */
} mag_cal_method_t;

typedef struct {
    float offset[3]; /* 硬磁偏移 µT */
    float soft[9];   /* 3x3 行主序：out = soft * (in - offset)，含软磁与三轴缩放 */
    float radius;    /* 校准后磁场平均半径 µT */
    float quality;   /* 0~1：三个轴转动幅度是否均衡，<0.4 说明没转开 */
    float residual;  /* 椭球拟合残差（归一化半径 RMS，0.01 = 1%）；min-max 时为 0 */
    uint8_t method;  /* mag_cal_method_t */
    bool valid;
} mag_cal_t;

/** 初始化：绑总线、探测 0x14/0x15、配置 100Hz/AVG4、进正常模式、从 NVS 载入校准 */
esp_err_t mag_init(i2c_master_bus_handle_t bus);

/** 传感器是否在线 */
bool mag_present(void);

/** 实际生效的 I2C 地址，不在线返回 0 */
uint8_t mag_address(void);

/** 取 Bosch 驱动句柄（诊断用；未初始化返回 NULL） */
struct bmm350_dev *mag_device(void);

/* ---------------- 轴向映射（运行时可配，存 NVS）---------------- */

/**
 * 取当前生效的轴向映射编号 0..23（24 是"真旋转"的全部可能）。
 * 优先用自动检测/手工设定的值；没设过则回退到编译期配置 APP_MAG_AXIS_*。
 */
int mag_axis_get(void);

/** 设置并持久化轴向映射（存 NVS） */
esp_err_t mag_axis_set(int idx);

/**
 * NVS 里是否已有"自动检测出来的"轴向映射。
 * 启动流程用它决定要不要重新做一次轴向检测。
 */
bool mag_axis_is_stored(void);

/**
 * 读一次数据（带 DRDY 检查）。
 * @param raw  原始补偿值（µT，未做硬磁校准），可为 NULL
 * @param cal  硬磁校准后的值（µT），可为 NULL
 * @param fresh 输出：本次是否是"新数据"（没有新数据时返回 true 但 fresh=false）
 * @return ESP_OK 读取成功；其它表示通信失败
 */
esp_err_t mag_read(float raw[3], float cal[3], bool *fresh);

/** 读取 BMM350 补偿后的传感器 XYZ，绕过当前轴向映射和硬磁校准（轴向检测用） */
esp_err_t mag_read_unmapped(float raw[3]);

/* ---------------- 硬磁校准 ---------------- */

/** 开始校准，duration_ms 内持续调用 mag_cal_process() 喂原始样本 */
void mag_cal_start(uint32_t duration_ms);
bool mag_cal_is_running(void);
float mag_cal_progress(void); /* 0~1 */

/**
 * 喂一个原始样本（机体系、已轴向映射）。到时间后自动结束：
 * 先用椭球拟合算 offset/soft，拟合失败则回退 min-max，结果存 NVS。
 * @return true = 本次调用刚好结束校准
 */
bool mag_cal_process(const float raw[3]);

/** 中途放弃校准（超时保护用）：清掉"正在校准"标志，不保存任何结果 */
void mag_cal_abort(void);

/** 是否已有可用校准数据（NVS 或本次运行） */
bool mag_is_calibrated(void);

/** 校准方法名称（日志用） */
const char *mag_cal_method_name(uint8_t method);

/** 取校准参数副本 */
void mag_cal_get(mag_cal_t *out);

/** 清除校准（内存 + NVS） */
esp_err_t mag_cal_clear(void);

/** 应用校准：out = soft * (in - offset) */
void mag_apply(const mag_cal_t *c, const float in[3], float out[3]);

#ifdef __cplusplus
}
#endif
