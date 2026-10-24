/*
 * mag_fit.h — 椭球拟合磁校准（纯浮点数学，不依赖 ESP-IDF）
 *
 * 模型：BMM350 读到的磁场 m 是真实地磁经过硬磁偏移 b 和软磁矩阵 A 之后的
 * 结果，即 m = A · m_true + b（A 对称正定，包含三轴灵敏度差异与交叉轴耦合）。
 * 因此 m 的端点落在一个椭球上：(m - b)ᵀ Q (m - b) = 1。
 *
 * 这里用最小二乘拟合一般二次曲面，再化成本包要用的形式：
 *     m_true ∝ soft · (m - center)
 * 也就是同时给出硬磁偏移（center）、三轴缩放与交叉轴解耦（soft）。
 *
 * 与旧的 min-max 法的区别：
 *   - min-max 只用每个轴的最大/最小值，等价于"轴对齐的盒子里塞一个球"，
 *     给不出交叉轴耦合，而且少转半个圈就会把半径估小；
 *   - 椭球拟合用全部样本约束一个统一的二次曲面，覆盖率不足会直接反映成
 *     残差变大，不会"看起来很自信地算错"。
 *
 * 为什么单独一个文件：这段是纯数学，可以在 PC 上用 gcc 单独编译测试
 * （见 tools/host_test/）。磁校准错在校准环节的话，后面所有 yaw 都错，
 * 而且极难从日志看出来，所以必须有主机侧的回归测试。
 */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** 拟合允许的最大样本数（超过时调用方应抽稀） */
#define MAG_FIT_MAX_SAMPLES 512

typedef struct {
    float center[3]; /* 椭球中心 = 硬磁偏移（µT） */
    float soft[9];   /* 3x3 行主序：out = soft * (in - center)，单位与输入相同 */
    float radius;    /* 校准后磁场半径（µT）：|out| 应约等于它 */
    float residual;  /* 归一化半径的 RMS 偏差，0.01 = 1% */
    int used;        /* 参与最终拟合的样本数（异常值已剔除） */
} mag_fit_result_t;

/**
 * 用 n 个样本（samples 为 n*3 个 float，xyz 依次排列）拟合椭球。
 *
 * @return true 拟合成功且结果通过合理性检查；false 表示样本不足、退化
 *         （几乎共面/共线）、二次曲面不是椭球（非正定）或结果明显离谱。
 *         失败时 out 内容未定义，调用方应回退到其它校准方法。
 *
 * 说明：样本少于 40 个一律失败；函数内部会先按质心/尺度归一化再解方程，
 * 所以 µT 量级的数据不会因为数值范围大而精度下降。
 */
bool mag_fit_ellipsoid(const float *samples, int n, mag_fit_result_t *out);

#ifdef __cplusplus
}
#endif
