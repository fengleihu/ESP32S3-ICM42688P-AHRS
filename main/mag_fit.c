#include "mag_fit.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/*
 * 拟合目标：a x² + b y² + c z² + 2d xy + 2e xz + 2f yz + 2g x + 2h y + 2i z = 1
 * 九个系数对样本是线性的，所以就是解一个 9x9 正规方程。
 *
 * 为什么先做"减质心、除尺度"再解：µT 量级的坐标直接代进 x²、xy 这些项，
 * 正规方程的条件数会差到 float 都存不住；归一化到 O(1) 之后用 double 解，
 * 512 个样本的精度完全够（实测中心误差 < 0.1 µT）。
 *
 * 解完之后由二次型系数反推几何量：
 *   Qm = [[a,d,e],[d,b,f],[e,f,c]]，v = [g,h,i]
 *   xᵀQm x + 2vᵀx = 1  <=>  (x-c)ᵀQm(x-c) = k，c = -Qm⁻¹v，k = 1 - vᵀc
 *   则 Q = Qm/k 满足 (x-c)ᵀQ(x-c) = 1
 *
 * 关键的一步：椭球本身只能定出"球心 + 形状"，定不出变换矩阵的**朝向**
 * ——任何 O·√Q（O 正交）都把同一椭球映到单位球、且 |O·√Q·(m-c)| 都等于 1。
 * 这个朝向自由度必须用物理假设消掉：软磁矩阵在通常模型里是**对称正定**的
 * （各向异性磁导率张量），因此它的逆也是对称正定的，校正矩阵也只能取
 * 对称正定那一支，即 M = √Q = V·diag(√λ)·Vᵀ。
 *
 * 这是第一版真实踩过的坑：用 Cholesky 分解 Q = LLᵀ 再拿 Lᵀ 当 M，中心
 * 和半径都对，但 M 里混进了一个任意旋转，实测在 ±60° 倾斜范围内航向误差
 * 均值 3.9°、最大 8.2°（tools/host_test 里有这个回归用例）。换成对称正定
 * 平方根后同样的数据降到 0.000°。
 *
 * 退化情形必须显式判掉：样本几乎共面（只在一个方向转了转）时 Qm 不定，
 * 解出来的可能是双曲面，这时中心会在几百 µT 外——直接拒绝，不要保存。
 */

#define FIT_N 9
#define MIN_SAMPLES 40
#define MAX_AXIS_RATIO 4.0 /* soft 三列范数最大/最小超过它说明椭球离谱 */
#define MAX_RESIDUAL 0.20  /* 归一化半径 RMS 超过它说明不像是同一个椭球 */
#define OUTLIER_MIN_THRESHOLD 0.03 /* 归一化半径上至少放过 3% 的波动 */

static double dot3(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static double norm3(const double a[3])
{
    return sqrt(dot3(a, a));
}

static double det3(const double m[3][3])
{
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
           m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
           m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

static bool inv3(const double m[3][3], double out[3][3])
{
    double d = det3(m);
    if (!(fabs(d) > 1e-18)) {
        return false;
    }
    double inv = 1.0 / d;
    out[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv;
    out[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv;
    out[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv;
    out[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv;
    out[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv;
    out[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv;
    out[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv;
    out[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv;
    out[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv;
    return true;
}

/*
 * 对称 3x3 的 Jacobi 特征分解：a 被就地对角化，v 的列是特征向量。
 * 3x3 用 Jacobi 比求特征多项式稳：不需要处理三次方程的判别式。
 */
static void jacobi3(double a[3][3], double v[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            v[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }

    for (int sweep = 0; sweep < 16; sweep++) {
        double off = fabs(a[0][1]) + fabs(a[0][2]) + fabs(a[1][2]);
        if (off < 1e-18) {
            break;
        }
        for (int p = 0; p < 2; p++) {
            for (int q = p + 1; q < 3; q++) {
                if (fabs(a[p][q]) < 1e-20) {
                    continue;
                }
                double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                double t = (theta >= 0.0 ? 1.0 : -1.0) /
                           (fabs(theta) + sqrt(theta * theta + 1.0));
                double c = 1.0 / sqrt(t * t + 1.0);
                double s = t * c;

                for (int k = 0; k < 3; k++) {
                    double akp = a[k][p], akq = a[k][q];
                    a[k][p] = c * akp - s * akq;
                    a[k][q] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; k++) {
                    double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = c * apk - s * aqk;
                    a[q][k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 3; k++) {
                    double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = c * vkp - s * vkq;
                    v[k][q] = s * vkp + c * vkq;
                }
            }
        }
    }
}

/* 高斯消元（列主元）。a 与 b 被就地破坏。 */
static bool solve9(double a[FIT_N][FIT_N], double b[FIT_N], double x[FIT_N])
{
    double amax = 0.0;
    for (int i = 0; i < FIT_N; i++) {
        for (int j = 0; j < FIT_N; j++) {
            double v = fabs(a[i][j]);
            if (v > amax) {
                amax = v;
            }
        }
    }
    if (!(amax > 0.0)) {
        return false;
    }
    const double eps = amax * 1e-13;

    for (int col = 0; col < FIT_N; col++) {
        int piv = col;
        double best = fabs(a[col][col]);
        for (int r = col + 1; r < FIT_N; r++) {
            double v = fabs(a[r][col]);
            if (v > best) {
                best = v;
                piv = r;
            }
        }
        if (best < eps) {
            return false; /* 奇异：样本不足以约束全部系数 */
        }
        if (piv != col) {
            for (int c = 0; c < FIT_N; c++) {
                double t = a[col][c];
                a[col][c] = a[piv][c];
                a[piv][c] = t;
            }
            double t = b[col];
            b[col] = b[piv];
            b[piv] = t;
        }
        const double d = a[col][col];
        for (int r = col + 1; r < FIT_N; r++) {
            double f = a[r][col] / d;
            if (f == 0.0) {
                continue;
            }
            for (int c = col; c < FIT_N; c++) {
                a[r][c] -= f * a[col][c];
            }
            b[r] -= f * b[col];
        }
    }

    for (int r = FIT_N - 1; r >= 0; r--) {
        double s = b[r];
        for (int c = r + 1; c < FIT_N; c++) {
            s -= a[r][c] * x[c];
        }
        x[r] = s / a[r][r];
    }
    return true;
}

/*
 * 用 idx[] 指定的子集做一次拟合，输出 center（µT）与 M（1/µT，单位球映射）。
 * 不含残差统计：调用方需要的是"全样本"口径的残差，由 residuals_all 算。
 */
static bool fit_once(const float *samples, const int *idx, int m, double center[3],
                     double M[3][3])
{
    if (m < FIT_N + 1) {
        return false;
    }

    /* 1) 质心与尺度 */
    double mu[3] = {0.0, 0.0, 0.0};
    for (int k = 0; k < m; k++) {
        const float *p = samples + 3 * idx[k];
        mu[0] += p[0];
        mu[1] += p[1];
        mu[2] += p[2];
    }
    for (int i = 0; i < 3; i++) {
        mu[i] /= (double)m;
    }

    double scale = 0.0;
    for (int k = 0; k < m; k++) {
        const float *p = samples + 3 * idx[k];
        double d[3] = {p[0] - mu[0], p[1] - mu[1], p[2] - mu[2]};
        scale += norm3(d);
    }
    scale /= (double)m;
    if (!(scale > 1e-6)) {
        return false; /* 所有样本几乎同一个点 */
    }

    /* 2) 正规方程 (AᵀA)p = Aᵀ·1 */
    double S[FIT_N][FIT_N];
    double rhs[FIT_N];
    memset(S, 0, sizeof(S));
    memset(rhs, 0, sizeof(rhs));
    for (int k = 0; k < m; k++) {
        const float *p = samples + 3 * idx[k];
        double u0 = (p[0] - mu[0]) / scale;
        double u1 = (p[1] - mu[1]) / scale;
        double u2 = (p[2] - mu[2]) / scale;
        double row[FIT_N] = {u0 * u0, u1 * u1, u2 * u2, 2.0 * u0 * u1,
                             2.0 * u0 * u2, 2.0 * u1 * u2, 2.0 * u0, 2.0 * u1,
                             2.0 * u2};
        for (int i = 0; i < FIT_N; i++) {
            rhs[i] += row[i];
            for (int j = 0; j < FIT_N; j++) {
                S[i][j] += row[i] * row[j];
            }
        }
    }

    double coef[FIT_N];
    if (!solve9(S, rhs, coef)) {
        return false;
    }

    /* 3) 二次型 -> 中心与 Q */
    double Qm[3][3] = {
        {coef[0], coef[3], coef[4]},
        {coef[3], coef[1], coef[5]},
        {coef[4], coef[5], coef[2]},
    };
    double v[3] = {coef[6], coef[7], coef[8]};

    double Qinv[3][3];
    if (det3(Qm) <= 0.0 || !inv3(Qm, Qinv)) {
        return false; /* 不是椭球（双曲面/抛物面）：样本覆盖不足或严重畸变 */
    }

    double c_u[3];
    for (int i = 0; i < 3; i++) {
        c_u[i] = -(Qinv[i][0] * v[0] + Qinv[i][1] * v[1] + Qinv[i][2] * v[2]);
    }
    double k = 1.0 - dot3(v, c_u);
    if (!(k > 1e-9)) {
        return false;
    }

    double Q[3][3];
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Q[i][j] = Qm[i][j] / k;
        }
    }

    /* 4) 对称正定平方根：M = V·diag(√λ)·Vᵀ / scale，center = mu + s·c_u */
    double V[3][3];
    jacobi3(Q, V);
    double sd[3];
    for (int i = 0; i < 3; i++) {
        if (!(Q[i][i] > 1e-18)) {
            return false; /* 特征值非正：不是椭球 */
        }
        sd[i] = sqrt(Q[i][i]);
    }
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            double s = 0.0;
            for (int t = 0; t < 3; t++) {
                s += V[i][t] * sd[t] * V[j][t];
            }
            M[i][j] = s / scale;
        }
        center[i] = mu[i] + scale * c_u[i];
    }
    return true;
}

/* 归一化半径 r_i = |M(p_i - center)|，合格拟合应全部≈1 */
static void residuals_all(const float *samples, int n, const double center[3],
                          const double M[3][3], double *r)
{
    for (int k = 0; k < n; k++) {
        const float *p = samples + 3 * k;
        double d[3] = {p[0] - center[0], p[1] - center[1], p[2] - center[2]};
        double t[3] = {
            M[0][0] * d[0] + M[0][1] * d[1] + M[0][2] * d[2],
            M[1][0] * d[0] + M[1][1] * d[1] + M[1][2] * d[2],
            M[2][0] * d[0] + M[2][1] * d[1] + M[2][2] * d[2],
        };
        r[k] = norm3(t);
    }
}

static int cmp_double(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return (x < y) ? -1 : (x > y ? 1 : 0);
}

static double median_inplace(double *v, int n)
{
    qsort(v, (size_t)n, sizeof(double), cmp_double);
    return (n & 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

/* 按偏差排序用的索引记录（要把"最差的 k 个"挑出来） */
typedef struct {
    double dev;
    int idx;
} dev_ent_t;

static int cmp_dev(const void *a, const void *b)
{
    const dev_ent_t *x = (const dev_ent_t *)a;
    const dev_ent_t *y = (const dev_ent_t *)b;
    return (x->dev < y->dev) ? -1 : (x->dev > y->dev ? 1 : 0);
}

bool mag_fit_ellipsoid(const float *samples, int n, mag_fit_result_t *out)
{
    if (!samples || !out || n < MIN_SAMPLES || n > MAG_FIT_MAX_SAMPLES) {
        return false;
    }

    int *cur = (int *)malloc(sizeof(int) * (size_t)n);   /* 本轮参与拟合 */
    int *next = (int *)malloc(sizeof(int) * (size_t)n);  /* 剔除后 */
    double *r = (double *)malloc(sizeof(double) * (size_t)n);
    double *scratch = (double *)malloc(sizeof(double) * (size_t)n);
    dev_ent_t *dev_ents = (dev_ent_t *)malloc(sizeof(dev_ent_t) * (size_t)n);
    bool ok = cur && next && r && scratch && dev_ents;
    if (!ok) {
        free(cur);
        free(next);
        free(r);
        free(scratch);
        free(dev_ents);
        return false;
    }

    int m = n;
    for (int i = 0; i < n; i++) {
        cur[i] = i;
    }

    double center[3], M[3][3];
    ok = fit_once(samples, cur, m, center, M);

    /* 剔除离群点：转动时偶尔会蹭到铁磁物，几帧 80 µT 的尖峰就足以把中心
     * 拖偏二十来 µT。
     *
     * 普通最小二乘对离群点的崩溃点是 0（一个坏点就能带偏），所以这里用
     * "最小修剪平方"（LTS）的两段式做法：
     *   阶段 A：每轮只保留残差最小的 h = 0.75n 个样本重新拟合，迭代几次。
     *           3% 的尖峰一定会落在这 75% 之外，拟合被拉回主群。
     *   阶段 B：用收敛后的参数算全部样本的残差，按 中位数 + 3×MAD 把主群
     *           的正常点都放回来，再做最后一次拟合。
     *
     * 为什么不能只做"3σ 之外剔掉"（第一版就是这么写的）：第一轮拟合已经被
     * 污染带偏，σ 本身被撑大，尖峰"看起来还在 3σ 以内"，迭代卡住不动
     * （实测剩余中心误差 21 µT）。硬修剪那一阶段是必要的。 */
    if (ok) {
        int h = (int)(0.75 * (double)n);
        if (h < MIN_SAMPLES) {
            h = MIN_SAMPLES;
        }
        if (h > n) {
            h = n;
        }

        for (int iter = 0; iter < 3 && h < n; iter++) {
            residuals_all(samples, n, center, M, r);
            memcpy(scratch, r, sizeof(double) * (size_t)n);
            double med = median_inplace(scratch, n);
            for (int i = 0; i < n; i++) {
                dev_ents[i].dev = fabs(r[i] - med);
                dev_ents[i].idx = i;
            }
            qsort(dev_ents, (size_t)n, sizeof(dev_ent_t), cmp_dev);
            for (int k = 0; k < h; k++) {
                next[k] = dev_ents[k].idx;
            }
            double c2[3], M2[3][3];
            if (!fit_once(samples, next, h, c2, M2)) {
                break;
            }
            memcpy(center, c2, sizeof(center));
            memcpy(M, M2, sizeof(M));
            m = h;
            int *tmp = cur;
            cur = next;
            next = tmp;
        }

        /* 阶段 B：阈值回放 */
        residuals_all(samples, n, center, M, r);
        memcpy(scratch, r, sizeof(double) * (size_t)n);
        double med = median_inplace(scratch, n);
        for (int i = 0; i < n; i++) {
            dev_ents[i].dev = fabs(r[i] - med);
            dev_ents[i].idx = i;
            scratch[i] = dev_ents[i].dev;
        }
        double thresh = 3.0 * median_inplace(scratch, n) * 1.4826;
        if (thresh < OUTLIER_MIN_THRESHOLD) {
            thresh = OUTLIER_MIN_THRESHOLD;
        }
        int cnt = 0;
        for (int i = 0; i < n; i++) {
            if (dev_ents[i].dev <= thresh) {
                next[cnt++] = i;
            }
        }
        if (cnt >= MIN_SAMPLES && cnt != m) {
            double c3[3], M3[3][3];
            if (fit_once(samples, next, cnt, c3, M3)) {
                memcpy(center, c3, sizeof(center));
                memcpy(M, M3, sizeof(M));
                m = cnt;
                int *tmp = cur;
                cur = next;
                next = tmp;
            }
        }
    }

    /* 最终残差统计（只在参与拟合的样本上算） */
    double mean = 0.0, rms = 0.0;
    if (ok) {
        residuals_all(samples, n, center, M, r);
        for (int k = 0; k < m; k++) {
            mean += r[cur[k]];
        }
        mean /= (double)m;
        for (int k = 0; k < m; k++) {
            double d = r[cur[k]] - mean;
            rms += d * d;
        }
        rms = sqrt(rms / (double)m);
    }

    /* ---- 合理性检查：宁可拒绝，也不要把一个错误的校准写进 NVS ---- */
    bool sane = ok && isfinite(center[0]) && isfinite(center[1]) &&
                isfinite(center[2]) && isfinite(mean) && isfinite(rms) &&
                rms <= MAX_RESIDUAL;

    double col_min = 1e30, col_max = 0.0;
    if (sane) {
        for (int j = 0; j < 3; j++) {
            double col[3] = {M[0][j], M[1][j], M[2][j]};
            double cn = norm3(col);
            if (cn < col_min) {
                col_min = cn;
            }
            if (cn > col_max) {
                col_max = cn;
            }
        }
        if (!(col_min > 0.0) || col_max / col_min > MAX_AXIS_RATIO) {
            sane = false;
        }
    }

    /* 平均物理半径：|p - center| 的均值，作为校准后的磁场模长 */
    double radius = 0.0;
    if (sane) {
        for (int k = 0; k < m; k++) {
            const float *p = samples + 3 * cur[k];
            double d[3] = {p[0] - center[0], p[1] - center[1], p[2] - center[2]};
            radius += norm3(d);
        }
        radius /= (double)m;
        if (!(radius >= 5.0 && radius <= 500.0)) {
            sane = false; /* 地磁场不可能这么小/大，说明拟合跑飞了 */
        }
    }

    if (sane) {
        for (int i = 0; i < 3; i++) {
            out->center[i] = (float)center[i];
        }
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                out->soft[i * 3 + j] = (float)(M[i][j] * radius);
            }
        }
        out->radius = (float)radius;
        out->residual = (float)rms;
        out->used = m;
    }

    free(cur);
    free(next);
    free(r);
    free(scratch);
    free(dev_ents);
    return sane;
}
