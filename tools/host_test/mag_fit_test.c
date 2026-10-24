/*
 * mag_fit_test.c — 椭球拟合的主机侧回归测试（gcc -O2 ... -lm）
 *
 * 为什么值得写：磁校准的输入是"人手转出来的样本"，错了也不会有异常日志，
 * 只会让所有 yaw 慢慢变歪。把拟合算法从 ESP-IDF 里剥出来（mag_fit.c 不依赖
 * 任何 IDF 头文件），就能在 PC 上用合成数据把精度量化出来：
 *
 *   1. 理想球（只有硬磁偏移）        -> 中心/半径误差应远小于 1 µT
 *   2. 带软磁（交叉轴耦合）          -> 中心恢复 + 航向误差
 *   3. 加噪声                        -> 中心误差仍 < 1 µT
 *   4. 3% 离群尖峰（蹭到铁磁物）      -> 剔除后不受影响
 *   5. 只转一小段弧（覆盖率不足）      -> 要么拒绝，要么不能假装很准
 *   6. 与旧 min-max 法对比航向误差   -> 这是换算法的直接收益，写进 HANDOFF
 *
 * 航向用的是固件里同一个公式（ahrs_mag_heading_deg 的倾斜补偿），
 * 所以这里算出来的误差就是"校准误差如何传导到 yaw"。
 *
 * 构建运行：
 *   gcc -O2 -Wall -Wextra -I main tools/host_test/mag_fit_test.c main/mag_fit.c \
 *       -lm -o tools/host_test/mag_fit_test
 *   ./tools/host_test/mag_fit_test
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mag_fit.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG (180.0 / M_PI)

/* ---------------- 确定性伪随机数（xorshift64*） ---------------- */
static unsigned long long g_rng = 0x9E3779B97F4A7C15ULL;

static void rng_seed(unsigned long long s)
{
    g_rng = s ? s : 1;
}

static double rng_uniform(void) /* [0,1) */
{
    g_rng ^= g_rng >> 12;
    g_rng ^= g_rng << 25;
    g_rng ^= g_rng >> 27;
    unsigned long long r = g_rng * 2685821657736338717ULL;
    return (double)(r >> 11) / 9007199254740992.0;
}

static double rng_sym(void) /* [-1,1) */
{
    return 2.0 * rng_uniform() - 1.0;
}

/* Box-Muller：单位方差正态 */
static double rng_normal(void)
{
    double u1 = rng_uniform();
    double u2 = rng_uniform();
    if (u1 < 1e-12) {
        u1 = 1e-12;
    }
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

/* ---------------- 向量/矩阵小工具 ---------------- */
static void mat_apply(const double m[9], const double v[3], double out[3])
{
    for (int i = 0; i < 3; i++) {
        out[i] = m[3 * i] * v[0] + m[3 * i + 1] * v[1] + m[3 * i + 2] * v[2];
    }
}

static double vnorm(const double v[3])
{
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void vunit(const double v[3], double out[3])
{
    double n = vnorm(v);
    if (n < 1e-12) {
        out[0] = out[1] = out[2] = 0.0;
        return;
    }
    for (int i = 0; i < 3; i++) {
        out[i] = v[i] / n;
    }
}

static double angle_between(const double a[3], const double b[3])
{
    double ua[3], ub[3];
    vunit(a, ua);
    vunit(b, ub);
    double d = ua[0] * ub[0] + ua[1] * ub[1] + ua[2] * ub[2];
    if (d > 1.0) {
        d = 1.0;
    }
    if (d < -1.0) {
        d = -1.0;
    }
    return acos(d) * DEG;
}

/* 均匀球面点 */
static void random_unit_sphere(double out[3])
{
    double z = rng_sym();
    double t = 2.0 * M_PI * rng_uniform();
    double r = sqrt(fmax(0.0, 1.0 - z * z));
    out[0] = r * cos(t);
    out[1] = r * sin(t);
    out[2] = z;
}

/* R = Rz(yaw)Ry(pitch)Rx(roll)：机体系 -> 世界系 */
static void rot_zyx(double roll_deg, double pitch_deg, double yaw_deg, double R[9])
{
    double cr = cos(roll_deg / DEG), sr = sin(roll_deg / DEG);
    double cp = cos(pitch_deg / DEG), sp = sin(pitch_deg / DEG);
    double cy = cos(yaw_deg / DEG), sy = sin(yaw_deg / DEG);

    R[0] = cy * cp;
    R[1] = cy * sp * sr - sy * cr;
    R[2] = cy * sp * cr + sy * sr;
    R[3] = sy * cp;
    R[4] = sy * sp * sr + cy * cr;
    R[5] = sy * sp * cr - cy * sr;
    R[6] = -sp;
    R[7] = cp * sr;
    R[8] = cp * cr;
}

/* ---------------- 与固件一致的倾斜补偿磁航向 ---------------- */
static double mag_heading_deg(const double accel[3], const double mag[3])
{
    double an = vnorm(accel), mn = vnorm(mag);
    if (an < 1e-9 || mn < 1e-9) {
        return 0.0;
    }
    double ax = accel[0] / an, ay = accel[1] / an, az = accel[2] / an;
    double mx = mag[0] / mn, my = mag[1] / mn, mz = mag[2] / mn;

    double pitch = asin(fmax(-1.0, fmin(1.0, -ax)));
    double roll = atan2(ay, az);

    double gx = -ax, gy = -ay, gz = -az;
    double d = mx * gx + my * gy + mz * gz;
    double hx = mx - d * gx, hy = my - d * gy, hz = mz - d * gz;

    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double v1 = cr * hy - sr * hz;
    double v2 = sr * hy + cr * hz;
    double tx = cp * hx + sp * v2;
    double ty = v1;
    return atan2(-ty, tx) * DEG;
}

static double angle_diff_deg(double a, double b)
{
    double d = a - b;
    while (d > 180.0) {
        d -= 360.0;
    }
    while (d < -180.0) {
        d += 360.0;
    }
    return d;
}

/* ---------------- 场景参数 ---------------- */
typedef struct {
    double A[9]; /* 软磁（对称正定） */
    double b[3]; /* 硬磁偏移 */
    double R;    /* 场强 */
    double noise;
} sensor_t;

static void sensor_make(sensor_t *s, const double A[9], const double b[3], double R,
                        double noise)
{
    memcpy(s->A, A, sizeof(s->A));
    memcpy(s->b, b, sizeof(s->b));
    s->R = R;
    s->noise = noise;
}

/* 真实机体系磁场 -> 传感器读数 */
static void sensor_read(const sensor_t *s, const double b_body[3], double out[3])
{
    mat_apply(s->A, b_body, out);
    for (int i = 0; i < 3; i++) {
        out[i] += s->b[i] + (s->noise > 0.0 ? s->noise * rng_normal() : 0.0);
    }
}

static void fill_sphere_samples(const sensor_t *s, float *buf, int n)
{
    for (int i = 0; i < n; i++) {
        double u[3];
        random_unit_sphere(u);
        double bb[3] = {s->R * u[0], s->R * u[1], s->R * u[2]};
        double m[3];
        sensor_read(s, bb, m);
        for (int k = 0; k < 3; k++) {
            buf[3 * i + k] = (float)m[k];
        }
    }
}

static void cal_apply(const mag_fit_result_t *f, const double in[3], double out[3])
{
    double d[3] = {in[0] - f->center[0], in[1] - f->center[1], in[2] - f->center[2]};
    for (int i = 0; i < 3; i++) {
        out[i] = f->soft[3 * i] * d[0] + f->soft[3 * i + 1] * d[1] +
                 f->soft[3 * i + 2] * d[2];
    }
}

/* 评估：在所有姿态下，校准误差造成的航向误差 */
static void heading_error_stats(const sensor_t *s, const mag_fit_result_t *f, int n,
                                double *mean_deg, double *max_deg)
{
    double sum = 0.0, mx = 0.0;
    for (int i = 0; i < n; i++) {
        double roll = rng_sym() * 60.0;
        double pitch = rng_sym() * 60.0;
        double yaw = rng_sym() * 180.0;
        double R[9];
        rot_zyx(roll, pitch, yaw, R);

        /* 世界系地磁：水平指向磁北，倾角 60°（中纬度典型值） */
        double bw[3] = {cos(60.0 / DEG), 0.0, -sin(60.0 / DEG)};
        double bb[3] = {
            R[0] * bw[0] + R[3] * bw[1] + R[6] * bw[2],
            R[1] * bw[0] + R[4] * bw[1] + R[7] * bw[2],
            R[2] * bw[0] + R[5] * bw[1] + R[8] * bw[2],
        };
        double aw[3] = {0.0, 0.0, -1.0};
        double ab[3] = {
            R[0] * aw[0] + R[3] * aw[1] + R[6] * aw[2],
            R[1] * aw[0] + R[4] * aw[1] + R[7] * aw[2],
            R[2] * aw[0] + R[5] * aw[1] + R[8] * aw[2],
        };

        double m[3];
        sensor_read(s, (double[3]){s->R * bb[0], s->R * bb[1], s->R * bb[2]}, m);

        double ref = mag_heading_deg(ab, (double[3]){s->R * bb[0], s->R * bb[1],
                                                     s->R * bb[2]});
        double corr[3];
        cal_apply(f, m, corr);
        double got = mag_heading_deg(ab, corr);

        double e = fabs(angle_diff_deg(got, ref));
        sum += e;
        if (e > mx) {
            mx = e;
        }
    }
    *mean_deg = sum / (double)n;
    *max_deg = mx;
}

/* 旧 min-max 法（与改造前 mag.c 完全一致），用于对照 */
typedef struct {
    double offset[3];
    double scale[3];
    double radius;
} minmax_t;

static minmax_t minmax_fit(const float *samples, int n)
{
    double mn[3] = {1e30, 1e30, 1e30}, mx[3] = {-1e30, -1e30, -1e30};
    for (int i = 0; i < n; i++) {
        for (int k = 0; k < 3; k++) {
            double v = samples[3 * i + k];
            if (v < mn[k]) {
                mn[k] = v;
            }
            if (v > mx[k]) {
                mx[k] = v;
            }
        }
    }
    minmax_t r;
    double half[3];
    for (int k = 0; k < 3; k++) {
        r.offset[k] = 0.5 * (mx[k] + mn[k]);
        half[k] = 0.5 * (mx[k] - mn[k]);
    }
    double avg = (half[0] + half[1] + half[2]) / 3.0;
    for (int k = 0; k < 3; k++) {
        r.scale[k] = half[k] > 1.0 ? avg / half[k] : 1.0;
    }
    r.radius = avg;
    return r;
}

static void minmax_heading_error_stats(const sensor_t *s, const minmax_t *mm, int n,
                                       double *mean_deg, double *max_deg)
{
    mag_fit_result_t f;
    memset(&f, 0, sizeof(f));
    memcpy(f.center, mm->offset, sizeof(f.center));
    f.soft[0] = (float)mm->scale[0];
    f.soft[4] = (float)mm->scale[1];
    f.soft[8] = (float)mm->scale[2];
    (void)s;
    heading_error_stats(s, &f, n, mean_deg, max_deg);
}

/* ---------------- 测试用例 ---------------- */
static int g_failures;

static void check(int cond, const char *what)
{
    if (cond) {
        printf("    [ok]   %s\n", what);
    } else {
        printf("    [FAIL] %s\n", what);
        g_failures++;
    }
}

static void print_result(const mag_fit_result_t *f)
{
    printf("    拟合: center=[%+8.2f %+8.2f %+8.2f] µT  radius=%7.2f 残留=%.3f%% "
           "样本=%d\n",
           f->center[0], f->center[1], f->center[2], f->radius,
           100.0 * f->residual, f->used);
}

int main(void)
{
    static float buf[MAG_FIT_MAX_SAMPLES * 3];
    mag_fit_result_t f;

    const double A_sphere[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const double A_soft[9] = {1.100, 0.060, 0.020, 0.060, 0.940, 0.030,
                              0.020, 0.030, 1.030};
    const double b_iron[3] = {25.0, -40.0, 12.0};

    /* ---- 1. 理想球：纯硬磁 ---- */
    printf("\n[1] 理想球（R=48µT，硬磁 [25,-40,12]，500 样本，无噪声）\n");
    rng_seed(1001);
    {
        sensor_t s;
        sensor_make(&s, A_sphere, b_iron, 48.0, 0.0);
        fill_sphere_samples(&s, buf, 500);
        bool ok = mag_fit_ellipsoid(buf, 500, &f);
        check(ok, "拟合成功");
        if (ok) {
            print_result(&f);
            double dc[3] = {f.center[0] - 25.0, f.center[1] + 40.0,
                            f.center[2] - 12.0};
            check(vnorm(dc) < 0.05, "中心误差 < 0.05 µT");
            check(fabs(f.radius - 48.0) < 0.2, "半径误差 < 0.2 µT");
            double mean, mx;
            heading_error_stats(&s, &f, 400, &mean, &mx);
            printf("    航向误差 mean=%.3f° max=%.3f°\n", mean, mx);
            check(mx < 0.1, "航向误差 < 0.1°");
        }
    }

    /* ---- 2. 软磁：交叉轴耦合 ---- */
    printf("\n[2] 软磁（对角 1.10/0.94/1.03 + 交叉轴 0.06/0.02/0.03）\n");
    rng_seed(2002);
    {
        sensor_t s;
        sensor_make(&s, A_soft, b_iron, 48.0, 0.0);
        fill_sphere_samples(&s, buf, 500);
        bool ok = mag_fit_ellipsoid(buf, 500, &f);
        check(ok, "拟合成功");
        if (ok) {
            print_result(&f);
            double dc[3] = {f.center[0] - 25.0, f.center[1] + 40.0,
                            f.center[2] - 12.0};
            check(vnorm(dc) < 0.1, "中心误差 < 0.1 µT");
            double mean, mx;
            heading_error_stats(&s, &f, 400, &mean, &mx);
            printf("    椭球拟合航向误差 mean=%.3f° max=%.3f°\n", mean, mx);
            check(mx < 0.2, "椭球拟合航向误差 < 0.2°");

            /* 校正后的磁场方向必须与真实机体系磁场方向一致 */
            rng_seed(7777);
            double worst = 0.0;
            for (int i = 0; i < 300; i++) {
                double u[3];
                random_unit_sphere(u);
                double bb[3] = {48.0 * u[0], 48.0 * u[1], 48.0 * u[2]};
                double raw[3];
                sensor_read(&s, bb, raw);
                double corr[3];
                cal_apply(&f, raw, corr);
                double e = angle_between(bb, corr);
                if (e > worst) {
                    worst = e;
                }
            }
            printf("    校正后磁场方向误差 max=%.3f°\n", worst);
            check(worst < 0.2, "方向误差 < 0.2°");

            minmax_t mm = minmax_fit(buf, 500);
            double mmean, mmx;
            minmax_heading_error_stats(&s, &mm, 400, &mmean, &mmx);
            printf("    min-max  航向误差 mean=%.3f° max=%.3f°（半径估计 %.1f µT）\n",
                   mmean, mmx, mm.radius);
            check(mx * 2.0 < mmx, "椭球拟合的最差误差不到 min-max 的一半");
        }
    }

    /* ---- 3. 噪声 ---- */
    printf("\n[3] 噪声 0.5 µT RMS（BMM350 典型量级）\n");
    rng_seed(3003);
    {
        sensor_t s;
        sensor_make(&s, A_soft, b_iron, 48.0, 0.5);
        fill_sphere_samples(&s, buf, 500);
        bool ok = mag_fit_ellipsoid(buf, 500, &f);
        check(ok, "拟合成功");
        if (ok) {
            print_result(&f);
            double dc[3] = {f.center[0] - 25.0, f.center[1] + 40.0,
                            f.center[2] - 12.0};
            check(vnorm(dc) < 0.6, "中心误差 < 0.6 µT");
            /* 评估时不加噪声：这里量的是"校准参数"本身的误差，
             * 单帧噪声（0.5/48 rad ≈ 0.6°）会把结论盖住。 */
            sensor_t se = s;
            se.noise = 0.0;
            double mean, mx;
            heading_error_stats(&se, &f, 400, &mean, &mx);
            printf("    校准后航向误差 mean=%.2f° max=%.2f°（评估无噪声）\n", mean,
                   mx);
            check(mx < 1.0, "航向误差 < 1°");
        }
    }

    /* ---- 4. 离群点 ---- */
    printf("\n[4] 3%% 离群尖峰（+80 µT，模拟蹭到铁磁物）\n");
    rng_seed(4004);
    {
        sensor_t s;
        sensor_make(&s, A_soft, b_iron, 48.0, 0.3);
        fill_sphere_samples(&s, buf, 500);
        for (int i = 0; i < 15; i++) {
            int k = (int)(rng_uniform() * 500.0) % 500;
            buf[3 * k] += 80.0f;
            buf[3 * k + 1] -= 60.0f;
        }
        bool ok = mag_fit_ellipsoid(buf, 500, &f);
        check(ok, "拟合成功");
        if (ok) {
            print_result(&f);
            double dc[3] = {f.center[0] - 25.0, f.center[1] + 40.0,
                            f.center[2] - 12.0};
            check(vnorm(dc) < 0.6, "中心误差 < 0.6 µT（离群点已剔除）");
            check(f.used <= 490, "确有样本被剔除");
        }
    }

    /* ---- 5. 覆盖不足：只在一个小锥角内转 ---- */
    printf("\n[5] 覆盖率不足（只在 25° 锥角内，200 样本）\n");
    rng_seed(5005);
    {
        sensor_t s;
        sensor_make(&s, A_soft, b_iron, 48.0, 0.3);
        for (int i = 0; i < 200; i++) {
            double u[3];
            random_unit_sphere(u);
            double cone = 25.0 / DEG;
            /* 收敛到 z 轴附近的锥内 */
            double ax_x = u[0] * sin(cone), ax_y = u[1] * sin(cone);
            double z = 1.0 - rng_uniform() * (1.0 - cos(cone));
            double r = sqrt(fmax(0.0, 1.0 - z * z));
            double n = sqrt(ax_x * ax_x + ax_y * ax_y);
            double dir[3];
            if (n < 1e-9) {
                dir[0] = 0;
                dir[1] = 0;
                dir[2] = 1;
            } else {
                dir[0] = r * ax_x / n;
                dir[1] = r * ax_y / n;
                dir[2] = z;
            }
            double bb[3] = {s.R * dir[0], s.R * dir[1], s.R * dir[2]};
            double m[3];
            sensor_read(&s, bb, m);
            for (int k = 0; k < 3; k++) {
                buf[3 * i + k] = (float)m[k];
            }
        }
        bool ok = mag_fit_ellipsoid(buf, 200, &f);
        if (ok) {
            print_result(&f);
            /* 只转一个 25° 锥角时，椭球在缺失的那一大片上完全没约束，
             * 拟合必然"成功"且残差很小，但中心可以偏出十几 µT。所以
             * 残差不是覆盖率判据——固件另外用 min/max 跨度（quality）
             * 拦这种情况。这里记录偏移量，证明门限是必要的。 */
            double dc[3] = {f.center[0] - 25.0, f.center[1] + 40.0,
                            f.center[2] - 12.0};
            printf("    中心偏差 %.1f µT —— 拟合本身看不出来，靠跨度门限兜底\n",
                   vnorm(dc));
            check(vnorm(dc) > 2.0, "覆盖不足确实会让中心偏出 >2 µT（必须靠 quality 门限）");
        } else {
            printf("    拟合被拒绝（同样可接受：宁可拒绝也不要错的校准）\n");
            check(1, "退化为拒绝");
        }
    }

    /* ---- 6. 只有 40 个样本（最小样本数边界） ---- */
    printf("\n[6] 边界：40 个样本\n");
    rng_seed(6006);
    {
        sensor_t s;
        sensor_make(&s, A_sphere, b_iron, 48.0, 0.0);
        fill_sphere_samples(&s, buf, 40);
        bool ok = mag_fit_ellipsoid(buf, 40, &f);
        check(ok, "40 样本仍可拟合");
        if (ok) {
            print_result(&f);
        }
        check(!mag_fit_ellipsoid(buf, 39, &f), "39 样本被拒绝");
    }

    printf("\n%s（失败 %d 项）\n", g_failures == 0 ? "全部通过" : "有失败项",
           g_failures);
    return g_failures == 0 ? 0 : 1;
}
