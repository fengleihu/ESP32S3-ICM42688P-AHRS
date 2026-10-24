#include "mag.h"

#include <math.h>
#include <string.h>

#include "bmm350.h"
#include "bmm350_common.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mag_axis.h"
#include "mag_fit.h"
#include "nvs.h"
#include "sdkconfig.h"

static const char *TAG = "mag";

#define MAG_NVS_NS       "att9axis"
#define MAG_NVS_KEY      "magcal"
#define MAG_NVS_AXIS_KEY "magaxis"
#define MAG_NVS_MAGIC    0x4D414732u /* "MAG2"：椭球拟合（offset + 3x3 soft） */
#define MAG_NVS_MAGIC_V1 0x4D414731u /* "MAG1"：旧的 min-max（offset + scale[3]） */

typedef struct {
    uint32_t magic;
    mag_cal_t cal;
} mag_nvs_blob_t;

/* 旧版 NVS 内容，只用于读取兼容（升级固件后不需要重新校准两次） */
typedef struct {
    float offset[3];
    float scale[3];
    float radius;
    float quality;
    bool valid;
} mag_cal_v1_t;

typedef struct {
    uint32_t magic;
    mag_cal_v1_t cal;
} mag_nvs_blob_v1_t;

static struct bmm350_dev s_dev;
static bool s_present;
static uint8_t s_addr;
static mag_cal_t s_cal;
static int s_axis = -1; /* 0..23；-1 表示还没从 NVS/config 解析出来 */

static bool s_cal_running;
static int64_t s_cal_start_us;
static uint32_t s_cal_duration_us;
static int s_cal_count;
static float s_min[3], s_max[3];

/* 椭球拟合用的样本缓存（静态：app_main 的栈只有几 KB，不能放局部数组）。
 * 15 秒 × 25 Hz ≈ 375 个样本，512 个槽够用；真跑更久就先抽稀一半再存，
 * 保证内存有上界。 */
static float s_samples[MAG_FIT_MAX_SAMPLES][3];
static int s_stored;
static uint32_t s_seen;
static uint32_t s_decim = 1;

const char *mag_cal_method_name(uint8_t method)
{
    switch (method) {
    case MAG_CAL_METHOD_ELLIPSOID:
        return "椭球拟合";
    case MAG_CAL_METHOD_MINMAX:
        return "min-max";
    default:
        return "无";
    }
}

/* ------------------------------------------------------------------ */
/* 轴向映射                                                            */
/*                                                                    */
/* 这一步不能省：BMM350 与 IMU 是两颗独立芯片，焊接方向大概率不一致。  */
/* 轴映射错了的表现是"yaw 一开始对、一倾斜就错"——因为磁力计的水平分量被   */
/* 错误地投影到了竖直轴上。第一版靠 APP_MAG_AXIS_PERM/INV 手工配，实测    */
/* Y 轴反了导致倾斜 45° 时 yaw 漂 34°；现在改成可由 mag_axis_detect()      */
/* 自动判定并持久化。                                                  */
/* ------------------------------------------------------------------ */

/* 把编译期配置（6 种排列 × 取反位）折成"24 旋转"里的一个索引。
 * 只在 NVS 里没有自动检测结果时用作回退。                                */
static int config_axis_to_rot(void)
{
    /* 直接按 APP_MAG_AXIS_PERM/INV 构造一个 3x3 变换，再在 24 个里找匹配。
     * 注意：如果配置出来的是"镜像"（行列式 -1），24 个真旋转里找不到，
     * 这时退回单位矩阵并告警 —— 镜像在物理上不可能，配错了。 */
    static const int8_t perm_of_cfg[6][9] = {
        {1, 0, 0, 0, 1, 0, 0, 0, 1}, /* XYZ */
        {1, 0, 0, 0, 0, 1, 0, 1, 0}, /* XZY */
        {0, 1, 0, 1, 0, 0, 0, 0, 1}, /* YXZ */
        {0, 1, 0, 0, 0, 1, 1, 0, 0}, /* YZX */
        {0, 0, 1, 1, 0, 0, 0, 1, 0}, /* ZXY */
        {0, 0, 1, 0, 1, 0, 1, 0, 0}, /* ZYX */
    };
    int p = CONFIG_APP_MAG_AXIS_PERM;
    if (p < 0 || p > 5) {
        p = 0;
    }
    int8_t want[9];
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            int8_t v = perm_of_cfg[p][r * 3 + c];
            /* 取反位：bit c 对应第 c 个源轴，等价于给矩阵第 c 列乘 -1 */
            if (CONFIG_APP_MAG_AXIS_INV & (1 << c)) {
                v = -v;
            }
            want[r * 3 + c] = v;
        }
    }

    for (int idx = 0; idx < 24; idx++) {
        float m[9];
        mag_axis_matrix(idx, m);
        bool same = true;
        for (int i = 0; i < 9; i++) {
            if ((int8_t)m[i] != want[i]) {
                same = false;
                break;
            }
        }
        if (same) {
            return idx;
        }
    }
    ESP_LOGW(TAG, "编译期轴向配置(perm=%d inv=%d)对应的是镜像变换，物理上不可能；"
                  "已退回单位矩阵。建议用自动检测", CONFIG_APP_MAG_AXIS_PERM,
             CONFIG_APP_MAG_AXIS_INV);
    return 23;
}

int mag_axis_get(void)
{
    if (s_axis >= 0) {
        return s_axis;
    }
    /* 首次调用：先看 NVS 里有没有自动检测结果 */
    nvs_handle_t h;
    int saved = -1;
    if (nvs_open(MAG_NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        int32_t v = -1;
        if (nvs_get_i32(h, MAG_NVS_AXIS_KEY, &v) == ESP_OK && v >= 0 && v < 24) {
            saved = (int)v;
        }
        nvs_close(h);
    }
    if (saved >= 0) {
        s_axis = saved;
        ESP_LOGI(TAG, "轴向映射：已从 NVS 载入 idx=%d", s_axis);
    } else {
        s_axis = config_axis_to_rot();
        ESP_LOGI(TAG, "轴向映射：用编译期配置 idx=%d", s_axis);
    }
    return s_axis;
}

esp_err_t mag_axis_set(int idx)
{
    if (idx < 0 || idx >= 24) {
        return ESP_ERR_INVALID_ARG;
    }
    s_axis = idx;
    nvs_handle_t h;
    esp_err_t err = nvs_open(MAG_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_i32(h, MAG_NVS_AXIS_KEY, idx);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    ESP_LOGI(TAG, "轴向映射已设为 idx=%d 并保存到 NVS", idx);
    return err;
}

bool mag_axis_is_stored(void)
{
    nvs_handle_t h;
    if (nvs_open(MAG_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }
    int32_t v = -1;
    bool found = (nvs_get_i32(h, MAG_NVS_AXIS_KEY, &v) == ESP_OK && v >= 0 && v < 24);
    nvs_close(h);
    return found;
}

static void axis_map(const float in[3], float out[3])
{
    mag_axis_apply(mag_axis_get(), in, out);
}

void mag_apply(const mag_cal_t *c, const float in[3], float out[3])
{
    if (!c || !c->valid) {
        out[0] = in[0];
        out[1] = in[1];
        out[2] = in[2];
        return;
    }
    const float d0 = in[0] - c->offset[0];
    const float d1 = in[1] - c->offset[1];
    const float d2 = in[2] - c->offset[2];
    out[0] = c->soft[0] * d0 + c->soft[1] * d1 + c->soft[2] * d2;
    out[1] = c->soft[3] * d0 + c->soft[4] * d1 + c->soft[5] * d2;
    out[2] = c->soft[6] * d0 + c->soft[7] * d1 + c->soft[8] * d2;
}

/* ------------------------------------------------------------------ */
/* NVS                                                                 */
/* ------------------------------------------------------------------ */
static void cal_load(void)
{
    nvs_handle_t h;
    if (nvs_open(MAG_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return;
    }

    size_t len = 0;
    if (nvs_get_blob(h, MAG_NVS_KEY, NULL, &len) == ESP_OK) {
        if (len == sizeof(mag_nvs_blob_t)) {
            mag_nvs_blob_t blob;
            size_t l = sizeof(blob);
            if (nvs_get_blob(h, MAG_NVS_KEY, &blob, &l) == ESP_OK &&
                blob.magic == MAG_NVS_MAGIC && blob.cal.valid) {
                s_cal = blob.cal;
                ESP_LOGI(TAG, "已载入磁力计校准（%s）：offset=[%.1f, %.1f, %.1f] µT "
                              "半径=%.1f µT 质量=%.2f 残差=%.2f%%",
                         mag_cal_method_name(s_cal.method), s_cal.offset[0],
                         s_cal.offset[1], s_cal.offset[2], s_cal.radius, s_cal.quality,
                         100.0f * s_cal.residual);
            }
        } else if (len == sizeof(mag_nvs_blob_v1_t)) {
            /* 旧版 min-max 校准：等价于 soft = diag(scale)，直接换算，不必重校准 */
            mag_nvs_blob_v1_t old;
            size_t l = sizeof(old);
            if (nvs_get_blob(h, MAG_NVS_KEY, &old, &l) == ESP_OK &&
                old.magic == MAG_NVS_MAGIC_V1 && old.cal.valid) {
                memset(&s_cal, 0, sizeof(s_cal));
                memcpy(s_cal.offset, old.cal.offset, sizeof(s_cal.offset));
                for (int i = 0; i < 3; i++) {
                    s_cal.soft[i * 4] = old.cal.scale[i];
                }
                s_cal.radius = old.cal.radius;
                s_cal.quality = old.cal.quality;
                s_cal.method = MAG_CAL_METHOD_MINMAX;
                s_cal.valid = true;
                ESP_LOGW(TAG, "已载入旧版 min-max 校准：offset=[%.1f, %.1f, %.1f] µT "
                              "半径=%.1f µT。它没有交叉轴软磁项，建议发 RECAL_MAG "
                              "重跑一次椭球拟合",
                         s_cal.offset[0], s_cal.offset[1], s_cal.offset[2],
                         s_cal.radius);
            }
        }
    }
    nvs_close(h);
}

static esp_err_t cal_save(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(MAG_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    mag_nvs_blob_t blob = {.magic = MAG_NVS_MAGIC, .cal = s_cal};
    err = nvs_set_blob(h, MAG_NVS_KEY, &blob, sizeof(blob));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */
esp_err_t mag_init(i2c_master_bus_handle_t bus)
{
    memset(&s_cal, 0, sizeof(s_cal));

    esp_err_t err = bmm350_set_i2c_master_bus_handle(bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "绑定 I2C 总线失败: %s（若为 NOT_SUPPORTED，请关闭 "
                      "CONFIG_I2C_BUS_BACKWARD_CONFIG）", esp_err_to_name(err));
        return err;
    }

    /* 依次尝试 0x14 / 0x15（ADSEL 接 GND 还是 VDD 决定地址） */
    const uint8_t try_addr[2] = {
        (uint8_t)CONFIG_APP_MAG_I2C_ADDR,
        (uint8_t)(CONFIG_APP_MAG_I2C_ADDR == 0x14 ? 0x15 : 0x14),
    };

    for (int i = 0; i < 2; i++) {
        uint8_t addr = try_addr[i];
        if (i2c_master_probe(bus, addr, 50) != ESP_OK) {
            ESP_LOGW(TAG, "0x%02X 无应答(NACK)，跳过", addr);
            continue;
        }

        bmm350_set_i2c_address(addr); /* 必须在 interface_init 之前 */
        if (bmm350_interface_init(&s_dev) != 0) {
            ESP_LOGW(TAG, "0x%02X interface_init 失败", addr);
            continue;
        }
        int8_t rslt = bmm350_init(&s_dev);
        if (rslt != BMM350_OK) {
            ESP_LOGE(TAG, "0x%02X 器件在，但 bmm350_init 失败，rslt=%d", addr, rslt);
            bmm350_error_codes_print_result("bmm350_init", rslt);
            bmm350_interface_deinit();
            continue;
        }

        s_present = true;
        s_addr = addr;
        ESP_LOGI(TAG, "BMM350 在线：地址=0x%02X chip_id=0x%02X", s_addr, s_dev.chip_id);

        int8_t r = 0;
        r |= bmm350_set_odr_performance(BMM350_DATA_RATE_100HZ, BMM350_AVERAGING_4, &s_dev);
        r |= bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &s_dev);
        r |= bmm350_set_powermode(BMM350_NORMAL_MODE, &s_dev);
        if (r != BMM350_OK) {
            ESP_LOGW(TAG, "部分配置写入失败（rslt=%d），仍继续尝试读取", r);
        }
        break;
    }

    if (!s_present) {
        ESP_LOGE(TAG, "BMM350 初始化失败。检查供电/接线，以及 CS 是否被拉低"
                      "（I2C 模式下 CS 必须为高）");
        return ESP_ERR_NOT_FOUND;
    }

#if CONFIG_APP_MAG_FORCE_RECAL
    ESP_LOGW(TAG, "APP_MAG_FORCE_RECAL=y：忽略 NVS 里的旧校准，本次重新校准");
    memset(&s_cal, 0, sizeof(s_cal));
    nvs_handle_t h;
    if (nvs_open(MAG_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, MAG_NVS_KEY);
        nvs_commit(h);
        nvs_close(h);
    }
#else
    cal_load();
#endif
    return ESP_OK;
}

bool mag_present(void)
{
    return s_present;
}

uint8_t mag_address(void)
{
    return s_present ? s_addr : 0;
}

struct bmm350_dev *mag_device(void)
{
    return s_present ? &s_dev : NULL;
}

static esp_err_t mag_read_internal(float raw[3], float cal[3], bool *fresh, bool apply_axis)
{
    if (fresh) {
        *fresh = false;
    }
    if (!s_present) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * 关于 DRDY：实测 BMM350 的 INT_STATUS(0x30) 里 DRDY 位**一直是 0**
     * （现场诊断确认过：PMU 状态正常、原始值在变、补偿值合理，唯独 DRDY 不置位）。
     * 原因是 DRDY 需要显式配置中断引脚/映射才会拉起来，不配置时它不会自己变。
     *
     * 所以这里**不拿 DRDY 做门控**：只把它当参考值返回，读数据本身无条件进行。
     * 代价是可能读到重复样本，但那比"永远读不到数据"好得多。
     * 真正的节流靠调用方的时间间隔（主循环 20Hz，与 25Hz 的有效更新率匹配）。
     */
    uint8_t drdy = 0;
    if (bmm350_get_interrupt_status(&drdy, &s_dev) != BMM350_OK) {
        drdy = 0; /* 读不到就当作没有，不影响下面继续读数据 */
    }

    struct bmm350_mag_temp_data d;
    if (bmm350_get_compensated_mag_xyz_temp_data(&d, &s_dev) != BMM350_OK) {
        return ESP_FAIL;
    }
    if (fresh) {
        *fresh = (drdy != 0);
    }

    float mapped[3];
    const float src[3] = {d.x, d.y, d.z};
    if (apply_axis) {
        axis_map(src, mapped);
    } else {
        memcpy(mapped, src, sizeof(mapped));
    }

    if (raw) {
        raw[0] = mapped[0];
        raw[1] = mapped[1];
        raw[2] = mapped[2];
    }
    if (cal) {
        mag_apply(&s_cal, mapped, cal);
    }
    return ESP_OK;
}

esp_err_t mag_read(float raw[3], float cal[3], bool *fresh)
{
    return mag_read_internal(raw, cal, fresh, true);
}

esp_err_t mag_read_unmapped(float raw[3])
{
    if (!raw) {
        return ESP_ERR_INVALID_ARG;
    }
    return mag_read_internal(raw, NULL, NULL, false);
}

/* ------------------------------------------------------------------ */
/* 磁校准：椭球拟合为主，min-max 兜底                                    */
/* ------------------------------------------------------------------ */
void mag_cal_start(uint32_t duration_ms)
{
    s_cal_running = true;
    s_cal_start_us = esp_timer_get_time();
    s_cal_duration_us = duration_ms * 1000u;
    s_cal_count = 0;
    s_stored = 0;
    s_seen = 0;
    s_decim = 1;
    for (int i = 0; i < 3; i++) {
        s_min[i] = 1e9f;
        s_max[i] = -1e9f;
    }
    ESP_LOGW(TAG, "磁力计校准开始：请在 %u 秒内把设备绕各轴慢慢转几圈（画 8 字）",
             (unsigned)(duration_ms / 1000));
}

bool mag_cal_is_running(void)
{
    return s_cal_running;
}

float mag_cal_progress(void)
{
    if (!s_cal_running || s_cal_duration_us == 0) {
        return 0.0f;
    }
    int64_t el = esp_timer_get_time() - s_cal_start_us;
    float p = (float)el / (float)s_cal_duration_us;
    return p < 0.0f ? 0.0f : (p > 1.0f ? 1.0f : p);
}

bool mag_cal_process(const float raw[3])
{
    if (!s_cal_running) {
        return false;
    }

    for (int i = 0; i < 3; i++) {
        if (raw[i] < s_min[i]) {
            s_min[i] = raw[i];
        }
        if (raw[i] > s_max[i]) {
            s_max[i] = raw[i];
        }
    }
    s_cal_count++;

    /* 存进拟合缓存；满了就抽稀一半（保留偶数号），把采样间隔翻倍。
     * 正常情况下 15 秒只有 375 个样本，永远走不到抽稀分支。 */
    if ((s_seen++ % s_decim) == 0) {
        if (s_stored < MAG_FIT_MAX_SAMPLES) {
            memcpy(s_samples[s_stored], raw, sizeof(float) * 3);
            s_stored++;
        } else if (s_decim < 64) {
            for (int i = 0; i < MAG_FIT_MAX_SAMPLES / 2; i++) {
                memcpy(s_samples[i], s_samples[2 * i], sizeof(float) * 3);
            }
            s_stored = MAG_FIT_MAX_SAMPLES / 2;
            s_decim *= 2;
        }
    }

    if (esp_timer_get_time() - s_cal_start_us < (int64_t)s_cal_duration_us) {
        return false;
    }

    /* ---- 结束 ---- */
    s_cal_running = false;

    float half[3];
    for (int i = 0; i < 3; i++) {
        if (s_max[i] <= s_min[i]) {
            ESP_LOGE(TAG, "校准失败：轴 %d 没有变化（是不是没转动？）", i);
            memset(&s_cal, 0, sizeof(s_cal));
            return true;
        }
        half[i] = (s_max[i] - s_min[i]) * 0.5f;
    }
    float avg = (half[0] + half[1] + half[2]) / 3.0f;
    if (avg < 5.0f) {
        ESP_LOGE(TAG, "校准失败：磁场变化太小（%.1f µT），确认转动幅度", avg);
        memset(&s_cal, 0, sizeof(s_cal));
        return true;
    }

    /* 覆盖率：三轴转动幅度越接近 1 越好。与校准方法无关，是独立的门限：
     * 只在一个方向晃一晃时，椭球拟合会"成功"但中心可以偏出十几 µT
     * （tools/host_test 的用例 5 就是这个），必须靠它拦下来。 */
    float mn = half[0] < half[1] ? half[0] : half[1];
    if (half[2] < mn) {
        mn = half[2];
    }
    float mx = half[0] > half[1] ? half[0] : half[1];
    if (half[2] > mx) {
        mx = half[2];
    }
    float quality = mx > 0.0f ? (mn / mx) : 0.0f;

    memset(&s_cal, 0, sizeof(s_cal));
    mag_fit_result_t fit;
    bool fitted = (s_stored >= 40) &&
                  mag_fit_ellipsoid(&s_samples[0][0], s_stored, &fit);

    if (fitted) {
        memcpy(s_cal.offset, fit.center, sizeof(s_cal.offset));
        memcpy(s_cal.soft, fit.soft, sizeof(s_cal.soft));
        s_cal.radius = fit.radius;
        s_cal.residual = fit.residual;
        s_cal.method = MAG_CAL_METHOD_ELLIPSOID;
        s_cal.valid = true;
        ESP_LOGI(TAG, "校准完成（椭球拟合）：样本 %d（拟合 %d，剔除 %d） "
                      "offset=[%.1f, %.1f, %.1f] µT 半径=%.1f µT 残差=%.2f%% 覆盖度=%.2f",
                 s_cal_count, s_stored, s_stored - fit.used, s_cal.offset[0],
                 s_cal.offset[1], s_cal.offset[2], s_cal.radius,
                 100.0f * s_cal.residual, quality);
        if (fit.used < (s_stored * 9) / 10) {
            ESP_LOGW(TAG, "超过 10%% 的样本被当成离群点剔除（%d/%d）：转动时是否"
                          "蹭到铁磁物了？", s_stored - fit.used, s_stored);
        }
    } else {
        /* 回退：min-max。只在椭球拟合失败（样本太少/退化）时才会走到这里，
         * 它的模型没有交叉轴软磁项，精度差一个量级。 */
        float offs[3];
        for (int i = 0; i < 3; i++) {
            offs[i] = (s_max[i] + s_min[i]) * 0.5f;
            s_cal.offset[i] = offs[i];
            s_cal.soft[i * 4] = (half[i] > 1.0f) ? (avg / half[i]) : 1.0f;
        }
        s_cal.radius = avg;
        s_cal.method = MAG_CAL_METHOD_MINMAX;
        s_cal.valid = true;
        ESP_LOGW(TAG, "椭球拟合不可用（%d 个样本），回退 min-max：offset=[%.1f, %.1f, "
                      "%.1f] µT 半径=%.1f µT",
                 s_stored, s_cal.offset[0], s_cal.offset[1], s_cal.offset[2],
                 s_cal.radius);
    }

    s_cal.quality = quality;
    if (s_cal.quality < 0.4f) {
        ESP_LOGW(TAG, "三轴转动幅度不均衡（覆盖度 %.2f），建议再转一次、"
                      "三个方向都转到位", s_cal.quality);
    }

    if (cal_save() != ESP_OK) {
        ESP_LOGW(TAG, "校准结果保存 NVS 失败（本次运行仍有效）");
    }
    return true;
}

bool mag_is_calibrated(void)
{
    return s_cal.valid;
}

void mag_cal_abort(void)
{
    if (s_cal_running) {
        ESP_LOGW(TAG, "校准被中止（未保存任何结果）");
    }
    s_cal_running = false;
}

void mag_cal_get(mag_cal_t *out)
{
    if (out) {
        *out = s_cal;
    }
}

esp_err_t mag_cal_clear(void)
{
    memset(&s_cal, 0, sizeof(s_cal));
    nvs_handle_t h;
    esp_err_t err = nvs_open(MAG_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_erase_key(h, MAG_NVS_KEY);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    ESP_LOGW(TAG, "已清除磁力计校准数据");
    return err;
}
