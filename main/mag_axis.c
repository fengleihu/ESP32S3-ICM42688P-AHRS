#include "mag_axis.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mag-axis";

/*
 * 24 种"真旋转"方位（旋转群的全部元素），行主序 3x3。
 *
 * 为什么是硬编码而不是用算法生成：第一版用"三个轴各转 0/90/180/270 再按 6 种
 * 顺序复合"来生成，结果 24 个里只有 16 个互不相同 —— 复合会重复覆盖同一批元素。
 * 这类 bug 烧到板子上的表现是"自动检测选中了错误映射"，而且极难归因。
 *
 * 这张表是在 PC 上用**穷举法**（从单位阵出发反复施加 6 个基本 90° 旋转，
 * 直到不再产生新元素）独立生成并验证过的：
 *   - 恰好 24 个元素
 *   - 行列式全为 +1（都是真旋转，不是镜像）
 *   - 互不相同
 *   - 全部是正交整数矩阵
 *
 * 索引 23 是单位矩阵（"轴向完全一致"）。
 */
static const int8_t ROT24[24][9] = {
    {-1, +0, +0, +0, -1, +0, +0, +0, +1},
    {-1, +0, +0, +0, +0, -1, +0, -1, +0},
    {-1, +0, +0, +0, +0, +1, +0, +1, +0},
    {-1, +0, +0, +0, +1, +0, +0, +0, -1},
    {+0, -1, +0, -1, +0, +0, +0, +0, -1},
    {+0, -1, +0, +0, +0, -1, +1, +0, +0},
    {+0, -1, +0, +0, +0, +1, -1, +0, +0},
    {+0, -1, +0, +1, +0, +0, +0, +0, +1},
    {+0, +0, -1, -1, +0, +0, +0, +1, +0},
    {+0, +0, -1, +0, -1, +0, -1, +0, +0},
    {+0, +0, -1, +0, +1, +0, +1, +0, +0},
    {+0, +0, -1, +1, +0, +0, +0, -1, +0},
    {+0, +0, +1, -1, +0, +0, +0, -1, +0},
    {+0, +0, +1, +0, -1, +0, +1, +0, +0},
    {+0, +0, +1, +0, +1, +0, -1, +0, +0},
    {+0, +0, +1, +1, +0, +0, +0, +1, +0},
    {+0, +1, +0, -1, +0, +0, +0, +0, +1},
    {+0, +1, +0, +0, +0, -1, -1, +0, +0},
    {+0, +1, +0, +0, +0, +1, +1, +0, +0},
    {+0, +1, +0, +1, +0, +0, +0, +0, -1},
    {+1, +0, +0, +0, -1, +0, +0, +0, -1},
    {+1, +0, +0, +0, +0, -1, +0, +1, +0},
    {+1, +0, +0, +0, +0, +1, +0, -1, +0},
    {+1, +0, +0, +0, +1, +0, +0, +0, +1},
};

/** 单位矩阵所在的索引（"轴向完全一致"） */
#define MAG_AXIS_IDENTITY_IDX 23

void mag_axis_matrix(int idx, float m[9])
{
    if (idx < 0 || idx >= 24) {
        idx = MAG_AXIS_IDENTITY_IDX;
    }
    for (int i = 0; i < 9; i++) {
        m[i] = (float)ROT24[idx][i];
    }
}

void mag_axis_apply(int idx, const float in[3], float out[3])
{
    float m[9];
    mag_axis_matrix(idx, m);
    out[0] = m[0] * in[0] + m[1] * in[1] + m[2] * in[2];
    out[1] = m[3] * in[0] + m[4] * in[1] + m[5] * in[2];
    out[2] = m[6] * in[0] + m[7] * in[1] + m[8] * in[2];
}

/*
 * 由加速度计求 roll/pitch（重力绝对可靠，不依赖磁力计）。
 * 这里刻意**不用 yaw**：如果拿滤波后的姿态去验证磁力计，就是自证循环。
 */
static void accel_to_rp(const float a[3], float *roll, float *pitch)
{
    float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (n < 1e-3f) {
        *roll = *pitch = 0.0f;
        return;
    }
    float ax = a[0] / n, ay = a[1] / n, az = a[2] / n;
    if (ax > 1.0f) {
        ax = 1.0f;
    }
    if (ax < -1.0f) {
        ax = -1.0f;
    }
    *pitch = asinf(-ax);
    *roll = atan2f(ay, az);
}

/* 机体系 -> 世界系，只用 roll/pitch：R = Ry(pitch) * Rx(roll) */
static void body_to_world_rp(float roll, float pitch, const float in[3], float out[3])
{
    float cr = cosf(roll), sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);

    float x1 = in[0];
    float y1 = cr * in[1] - sr * in[2];
    float z1 = sr * in[1] + cr * in[2];

    out[0] = cp * x1 + sp * z1;
    out[1] = y1;
    out[2] = -sp * x1 + cp * z1;
}

int mag_axis_detect(uint32_t seconds, mag_axis_sample_cb_t cb)
{
    if (!cb) {
        return -1;
    }

    /* 样本放静态区：2 × 1200 × 3 float = 28.8 KB，不能放栈上 */
    static float s_acc[MAG_AXIS_MAX_SAMPLES][3];
    static float s_mag[MAG_AXIS_MAX_SAMPLES][3];

    ESP_LOGW(TAG, "轴向自动检测开始：请在 %u 秒内**缓慢倾斜**设备到各个方向（不必转整圈）",
             (unsigned)seconds);

    int n = 0;
    int64_t t0 = esp_timer_get_time();
    int64_t next_sample = t0;
    int64_t end = t0 + (int64_t)seconds * 1000000;

    while (esp_timer_get_time() < end && n < MAG_AXIS_MAX_SAMPLES) {
        int64_t now = esp_timer_get_time();
        if (now < next_sample) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        next_sample = now + 20000; /* 50Hz */
        if (cb(s_acc[n], s_mag[n])) {
            n++;
        }
    }

    ESP_LOGI(TAG, "采集完成，共 %d 组样本", n);
    if (n < 60) {
        ESP_LOGE(TAG, "样本太少（%d），无法判定。请慢慢转、转满整段时长", n);
        return -1;
    }

    /*
     * 先做一次"数据有没有信息量"的自检。
     *
     * 踩过的坑：如果设备在采集期间基本不动，所有候选映射算出来的航向都是
     * 一条平直线，残差全为 0，24 个候选同分，于是"随便选一个"而且看起来
     * 还很自信（分数 0.00000）。这种**无声的失败**最危险 —— 用户以为成功了，
     * 其实选的是噪声。
     *
     * 所以先检查加速度计读数的方向跨度够不够大；不够就明确报错、不返回结果。
     */
    float acc_min[3] = {1e9f, 1e9f, 1e9f}, acc_max[3] = {-1e9f, -1e9f, -1e9f};
    for (int i = 0; i < n; i++) {
        for (int k = 0; k < 3; k++) {
            if (s_acc[i][k] < acc_min[k]) {
                acc_min[k] = s_acc[i][k];
            }
            if (s_acc[i][k] > acc_max[k]) {
                acc_max[k] = s_acc[i][k];
            }
        }
    }
    float span0 = acc_max[0] - acc_min[0];
    float span1 = acc_max[1] - acc_min[1];
    float span2 = acc_max[2] - acc_min[2];
    float span_total = span0 + span1 + span2;
    ESP_LOGI(TAG, "姿态激励自检：加速度三轴跨度 = %.2f / %.2f / %.2f g（合计 %.2f）",
             span0, span1, span2, span_total);

    /*
     * 静止时加速度恒为 (0,0,±1)，跨度≈0；倾斜到各个方向时每个轴都会在
     * -1..+1 之间走。阈值 1.2 大约对应"每个轴至少倾斜 40°以上"。
     */
    if (span_total < 1.2f) {
        ESP_LOGE(TAG, "姿态变化太小（合计 %.2f g，需要 > 1.2）—— 采集期间设备基本没动，"
                      "本次检测结果无效！", span_total);
        ESP_LOGE(TAG, "请重新上电，在提示出现后把设备**缓慢倾斜**到各个方向"
                      "（前后左右各倾 45° 以上，再翻过来朝下）");
        return -1;
    }

    float mag_min[3] = {1e9f, 1e9f, 1e9f};
    float mag_max[3] = {-1e9f, -1e9f, -1e9f};
    for (int i = 0; i < n; i++) {
        for (int k = 0; k < 3; k++) {
            if (s_mag[i][k] < mag_min[k]) {
                mag_min[k] = s_mag[i][k];
            }
            if (s_mag[i][k] > mag_max[k]) {
                mag_max[k] = s_mag[i][k];
            }
        }
    }
    float mag_offset[3];
    for (int k = 0; k < 3; k++) {
        mag_offset[k] = 0.5f * (mag_min[k] + mag_max[k]);
    }
    ESP_LOGI(TAG, "检测前去除采样磁场中心（硬磁偏移估计）：[%.1f, %.1f, %.1f] uT",
             mag_offset[0], mag_offset[1], mag_offset[2]);

    /*
     * 打分：用"磁航向随姿态的非线性漂移"作为判据。
     *
     * 这一版是第三版，前两版都失败过，值得记下为什么：
     *
     * 第一版：世界系磁场方向的方差。
     *   -> 24 个候选分数全部相同（0.00011），完全无法区分。那个判据只约束了
     *      磁场的竖直分量，水平方位角是自由的。
     *
     * 第二版：磁倾角 dip 的方差。
     *   -> 只能识别 10/24。因为绕**竖直轴**的旋转既不改变 dip、也不改变
     *      世界系磁场方向，数学上存在等价类，任何只看竖直分量的判据都分不开。
     *
     * 第三版（本版）：倾斜时磁航向 vs 时间的漂移。
     *   真正的错误是"倾斜时 yaw 会跟着乱变"，而绕竖直轴的旋转只带来一个
     *   **固定航向偏移**（等价于罗盘装歪了），无害。
     *
     * 关键处理：减掉线性趋势。
     *   用户转动时磁航向本来就会随时间线性变化；陀螺零偏也会造成线性漂移。
     *   这两种都表现为直线，不该被算作错误。错误轴向造成的是**非线性摆动**。
     *   所以用"航向减去最小二乘直线后的残差"来打分：
     *     正确轴向 -> 残差接近 0
     *     错误轴向 -> 残差很大
     */
    float scores[24];
    float best_score = 1e30f;
    int best_idx = -1;

    for (int idx = 0; idx < 24; idx++) {
        static float hdg[MAG_AXIS_MAX_SAMPLES];
        int valid = 0;

        for (int i = 0; i < n; i++) {
            float centered[3], mapped[3], world[3], roll, pitch;
            for (int k = 0; k < 3; k++) {
                centered[k] = s_mag[i][k] - mag_offset[k];
            }
            mag_axis_apply(idx, centered, mapped);
            accel_to_rp(s_acc[i], &roll, &pitch);
            body_to_world_rp(roll, pitch, mapped, world);

            float horiz = sqrtf(world[0] * world[0] + world[1] * world[1]);
            if (horiz < 1e-3f) {
                continue;
            }
            /* 与 ahrs_mag_heading_deg 同一套约定 */
            hdg[valid++] = atan2f(-world[1], world[0]);
        }
        if (valid < 30) {
            scores[idx] = 1e30f;
            continue;
        }

        /* 航向是角度，需要解缠绕，否则 ±180 处会出现假跳变 */
        for (int i = 1; i < valid; i++) {
            float d = hdg[i] - hdg[i - 1];
            while (d > (float)M_PI) {
                hdg[i] -= 2.0f * (float)M_PI;
                d = hdg[i] - hdg[i - 1];
            }
            while (d < -(float)M_PI) {
                hdg[i] += 2.0f * (float)M_PI;
                d = hdg[i] - hdg[i - 1];
            }
        }

        /* 最小二乘拟合直线 hdg ≈ a*i + b，只看残差 */
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int i = 0; i < valid; i++) {
            sx += i;
            sy += hdg[i];
            sxx += (double)i * i;
            sxy += (double)i * hdg[i];
        }
        double denom = valid * sxx - sx * sx;
        double a = 0, b = 0;
        if (fabs(denom) > 1e-9) {
            a = (valid * sxy - sx * sy) / denom;
            b = (sy - a * sx) / valid;
        } else {
            b = sy / valid;
        }

        double var = 0;
        for (int i = 0; i < valid; i++) {
            double r = hdg[i] - (a * i + b);
            var += r * r;
        }
        var /= valid;
        scores[idx] = (float)var;
        if (var < best_score) {
            best_score = (float)var;
            best_idx = idx;
        }
    }

    if (best_idx < 0) {
        ESP_LOGE(TAG, "所有候选都无效，放弃");
        return -1;
    }

    /* 打印前 5 名 */
    ESP_LOGI(TAG, "候选映射得分（越小越好），前 5 名：");
    float tmp[24];
    memcpy(tmp, scores, sizeof(tmp));
    for (int rank = 0; rank < 5; rank++) {
        int bi = -1;
        float bv = 1e30f;
        for (int i = 0; i < 24; i++) {
            if (tmp[i] < bv) {
                bv = tmp[i];
                bi = i;
            }
        }
        if (bi < 0) {
            break;
        }
        float m[9];
        mag_axis_matrix(bi, m);
        ESP_LOGI(TAG, "  %d位 idx=%2d 分数=%.5f 矩阵[%+.0f%+.0f%+.0f %+.0f%+.0f%+.0f %+.0f%+.0f%+.0f]",
                 rank + 1, bi, bv, m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
        tmp[bi] = 1e30f;
    }

    ESP_LOGI(TAG, "判定最佳映射 idx=%d（分数 %.5f）", best_idx, best_score);
    if (best_idx == MAG_AXIS_IDENTITY_IDX) {
        ESP_LOGI(TAG, "最佳映射是单位矩阵 —— 磁力计轴向与六轴完全一致，无需变换");
    }
    if (best_score > 0.02f) {
        ESP_LOGW(TAG, "最佳分数偏高（%.4f）：可能转动不充分、或附近有磁干扰。"
                      "建议换个远离金属的位置重试", best_score);
        ESP_LOGE(TAG, "轴向检测置信度不足，本次不应用候选映射");
        return -1;
    }
    return best_idx;
}
