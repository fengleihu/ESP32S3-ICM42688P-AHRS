/*
 * ahrs.h — Mahony 互补滤波 AHRS（6 轴：陀螺 + 加速度计；9 轴：再加磁力计）
 *
 * 纯 C99 + math.h，不依赖 ESP-IDF，便于在 PC 上做单元测试。
 * 本文件从 ESP32S3-MPU6050-AHRS 工程原样搬入（那份代码已在真机验证：
 * 静止 ROLL/PITCH σ = 0.008~0.010°，陀螺零偏校准后 YAW 漂移 0.22°/分钟）。
 * 数学部分一个字符都没改。
 *
 * 坐标系约定（与机体坐标系一致，右手系）：
 *   +X 前 / +Y 左 / +Z 上（模组水平放置、芯片面朝上时 az ≈ +1g）
 *
 * 输出欧拉角约定（与航空惯例一致，ZYX 顺序）：
 *   roll  : 绕 +X 旋转，右倾为正（右手定则）
 *   pitch : 绕 +Y 旋转，机头朝下为正
 *   yaw   : 绕 +Z 旋转，逆时针（从上往下看）为正
 *
 * 6 轴时 yaw 会缓慢漂移（数学上不可观测）—— 本工程只有 6 轴，
 * 所以屏幕上画的是"重力对准的立方体"，yaw 只做显示不做基准。
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AHRS_DEG2RAD 0.017453292519943295f
#define AHRS_RAD2DEG 57.29577951308232f

typedef struct {
    float q[4];        /* 四元数 w, x, y, z（机体 -> 世界） */
    float integral[3]; /* 误差积分项（体坐标系） */
    float kp;          /* 比例增益 */
    float ki;          /* 积分增益 */
} ahrs_t;

/** 初始化：kp/ki 为 Mahony 增益，典型 kp=2.0, ki=0.005 */
void ahrs_init(ahrs_t *a, float kp, float ki);

/** 重置姿态为单位四元数（清除积分项） */
void ahrs_reset(ahrs_t *a);

/**
 * 初始对准：直接用加速度计（重力）解出初始 roll/pitch 写入四元数。
 * 没有磁力计时 yaw 置 0。
 * 为什么必须做：Mahony 的误差反馈是"小角近似"，初始姿态偏差太大时收敛慢。
 */
bool ahrs_init_from_accel_mag(ahrs_t *a, const float accel[3], const float mag[3]);

/**
 * 6 轴更新（只用陀螺 + 加速度计，yaw 会漂移）。
 * @param gyro   角速度，单位 rad/s，体坐标系
 * @param accel  加速度，单位 g（任意一致比例即可，内部会归一化）
 * @param dt     距上次调用的时间，单位秒
 * @param trust_accel  加速度计可信度 0~1（1=完全信任），运动加速度大时传小值
 */
void ahrs_update(ahrs_t *a, const float gyro[3], const float accel[3],
                 float dt, float trust_accel);

/**
 * 9 轴更新（陀螺 + 加速度计 + 磁力计），yaw 锁定磁北、不漂移。
 *
 * 实现上就是"6 轴误差 + 磁力计误差"一起反馈，所以**每周期都要调用**：
 * 没有新磁力计数据时把 trust_mag 传 0 即可退化成 6 轴，不要跳过调用，
 * 否则加速度计的重力约束也会一起丢。
 *
 * @param mag    磁场强度，单位任意（µT 即可，内部会归一化），体坐标系，已做硬磁校准
 * @param trust_mag  磁力计可信度 0~1（附近有磁铁/大电流时传小值或 0）
 */
void ahrs_update_mag(ahrs_t *a, const float gyro[3], const float accel[3],
                     const float mag[3], float dt, float trust_accel, float trust_mag);

/** 取欧拉角（弧度）。任一指针可为 NULL。 */
void ahrs_get_euler(const ahrs_t *a, float *roll, float *pitch, float *yaw);

/** 取欧拉角（度）。任一指针可为 NULL。 */
void ahrs_get_euler_deg(const ahrs_t *a, float *roll, float *pitch, float *yaw);

/**
 * 单位四元数到旋转矩阵（行主序 3x3）。
 * 方向是"机体 -> 世界"：把机体坐标下的向量转到世界坐标。
 * 画立方体时我们用的是它的转置（世界系顶点 -> 机体系），见 cube.c。
 */
void ahrs_quat_to_matrix(const float q[4], float m[9]);

/**
 * 倾斜补偿磁航向（度）：只用加速度计 + 磁力计独立算出的航向。
 * 用于初始对准（让 yaw 一开始就是真实磁北航向，而不是从 0 爬过去）。
 * @return 航向角，正=俯视逆时针（与 yaw 同约定）；数据无效时返回 0 且 *valid=false
 */
float ahrs_mag_heading_deg(const float accel[3], const float mag[3], bool *valid);

#ifdef __cplusplus
}
#endif
