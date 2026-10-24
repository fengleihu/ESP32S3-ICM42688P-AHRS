/*
 * mag_axis.h — 磁力计轴向自动检测
 *
 * 为什么需要自动检测：BMM350 和 IMU 是两颗独立芯片，焊接方向可能不同。
 * 轴向映射错了的后果很具体——**倾斜时 yaw 会跟着漂**，因为磁场的竖直分量被
 * 当成水平分量用了。手工试错有 24 种合法组合（3 个轴的正交排列 × 8 种符号中
 * 构成"真旋转"的那 24 个），靠人逐一试倾斜测试不现实。
 *
 * 原理（这是唯一的硬约束）：
 *   地球磁场在"世界系"里是一个固定矢量 —— 板子怎么转，它在世界系里都不变。
 *   所以把磁力计读数按某个候选映射旋转、再用姿态把它转到世界系：
 *     - 映射正确  -> 不同姿态算出的世界系磁场**方向一致**
 *     - 映射错误  -> 世界系磁场随姿态乱转
 *   于是"让世界系磁场方差最小"的那个映射就是正确答案。
 *
 * 注意：姿态只取加速度计给出的 roll/pitch（重力绝对可靠），
 * yaw 不参与 —— 否则就变成自证了（用滤波后的姿态去验证磁力计）。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 采样回调：返回 true 表示取到一组有效数据 */
typedef bool (*mag_axis_sample_cb_t)(float accel[3], float mag[3]);

/** 最多收集多少组样本 */
#define MAG_AXIS_MAX_SAMPLES 1200

/**
 * 采集样本并自动判定最佳轴向映射。
 *
 * @param seconds  采集时长（建议 20 秒，期间请绕各方向转动设备）
 * @param cb       采样回调
 * @return 最佳映射编号 0..23；样本不足时返回 -1
 *
 * 判定结果会打印每种候选的分数，便于判断"是不是真的有个明显的赢家"。
 */
int mag_axis_detect(uint32_t seconds, mag_axis_sample_cb_t cb);

/**
 * 把映射编号转成矩阵（行主序 3x3）。越界时返回单位矩阵。
 * out = M * in
 */
void mag_axis_matrix(int idx, float m[9]);

/**
 * 用映射编号变换一个矢量：out = M * in
 */
void mag_axis_apply(int idx, const float in[3], float out[3]);

#ifdef __cplusplus
}
#endif
