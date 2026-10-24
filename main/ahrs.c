#include "ahrs.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD2DEG (180.0f / (float)M_PI)
#define DEG2RAD ((float)M_PI / 180.0f)

static float clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static void quat_normalize(float q[4])
{
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n < 1e-9f) {
        q[0] = 1.0f;
        q[1] = q[2] = q[3] = 0.0f;
        return;
    }
    float inv = 1.0f / n;
    q[0] *= inv;
    q[1] *= inv;
    q[2] *= inv;
    q[3] *= inv;
}

void ahrs_init(ahrs_t *a, float kp, float ki)
{
    memset(a, 0, sizeof(*a));
    a->q[0] = 1.0f;
    a->kp = kp;
    a->ki = ki;
}

void ahrs_reset(ahrs_t *a)
{
    a->q[0] = 1.0f;
    a->q[1] = a->q[2] = a->q[3] = 0.0f;
    a->integral[0] = a->integral[1] = a->integral[2] = 0.0f;
}

/* 误差反馈 + 四元数积分（6 轴 / 9 轴共用） */
static void ahrs_update_core(ahrs_t *a, const float gyro[3],
                             float ex, float ey, float ez, float dt)
{
    if (dt <= 0.0f) {
        return;
    }
    if (dt > 0.1f) {
        dt = 0.1f; /* 任务被长时间阻塞时不要一次积分过头 */
    }

    /* 误差积分（消除陀螺零偏） */
    if (a->ki > 0.0f) {
        a->integral[0] += a->ki * ex * dt;
        a->integral[1] += a->ki * ey * dt;
        a->integral[2] += a->ki * ez * dt;
    }

    /* 修正后的角速度（rad/s） */
    float gx = gyro[0] + a->kp * ex + a->integral[0];
    float gy = gyro[1] + a->kp * ey + a->integral[1];
    float gz = gyro[2] + a->kp * ez + a->integral[2];

    const float q0 = a->q[0], q1 = a->q[1], q2 = a->q[2], q3 = a->q[3];

    /* 四元数积分 q' = 0.5 * q ⊗ (0, gx, gy, gz) */
    float q0d = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float q1d = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float q2d = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float q3d = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    a->q[0] = q0 + q0d * dt;
    a->q[1] = q1 + q1d * dt;
    a->q[2] = q2 + q2d * dt;
    a->q[3] = q3 + q3d * dt;
    quat_normalize(a->q);
}

/* 加速度计误差：实测重力方向 × 估计重力方向（体坐标系） */
static void accel_error(const ahrs_t *a, const float accel[3], float trust,
                        float *ex, float *ey, float *ez)
{
    *ex = *ey = *ez = 0.0f;

    float ax = accel[0], ay = accel[1], az = accel[2];
    float an = sqrtf(ax * ax + ay * ay + az * az);
    if (an < 1e-3f) {
        return; /* 自由落体/无有效重力参考 */
    }
    float inv = 1.0f / an;
    ax *= inv;
    ay *= inv;
    az *= inv;

    const float q0 = a->q[0], q1 = a->q[1], q2 = a->q[2], q3 = a->q[3];

    /* 体坐标系下重力方向的估计值 v = R^T * (0,0,1) */
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float t = clampf(trust, 0.0f, 1.0f);
    *ex = (ay * vz - az * vy) * t;
    *ey = (az * vx - ax * vz) * t;
    *ez = (ax * vy - ay * vx) * t;
}

void ahrs_update(ahrs_t *a, const float gyro[3], const float accel[3],
                 float dt, float trust_accel)
{
    float ex, ey, ez;
    accel_error(a, accel, trust_accel, &ex, &ey, &ez);
    ahrs_update_core(a, gyro, ex, ey, ez, dt);
}

/*
 * 磁力计误差：把实测磁场与"世界系磁场只落在 X-Z 平面"这一约束比较。
 *
 * 直觉解释：世界系里磁北沿 +X，地球磁场除了水平分量还有竖直分量（磁倾角），
 * 但它**不会**出现在世界系的 Y 方向。把实测磁场转到世界系，Y 分量不为 0 的部分
 * 就是航向误差，反馈回去修正 yaw。
 *
 * 这也解释了 6 轴为什么治不了 yaw 漂移：没有磁力计时，绕 Z 轴旋转不改变重力方向
 * （重力在世界系不随 yaw 变化），所以 yaw 没有任何可观测参考，漂移是必然的。
 */
static void mag_error(const ahrs_t *a, const float mag[3], float trust,
                      float *ex, float *ey, float *ez)
{
    *ex = *ey = *ez = 0.0f;

    float t = clampf(trust, 0.0f, 1.0f);
    if (t <= 0.0f) {
        return;
    }

    float mx = mag[0], my = mag[1], mz = mag[2];
    float mn = sqrtf(mx * mx + my * my + mz * mz);
    if (mn < 1e-3f) {
        return;
    }
    float inv = 1.0f / mn;
    mx *= inv;
    my *= inv;
    mz *= inv;

    const float q0 = a->q[0], q1 = a->q[1], q2 = a->q[2], q3 = a->q[3];

    /* 把实测磁场旋转到世界系：h = R * m */
    float hx = 2.0f * (mx * (0.5f - q2 * q2 - q3 * q3) + my * (q1 * q2 - q0 * q3) +
                       mz * (q1 * q3 + q0 * q2));
    float hy = 2.0f * (mx * (q1 * q2 + q0 * q3) + my * (0.5f - q1 * q1 - q3 * q3) +
                       mz * (q2 * q3 - q0 * q1));

    /* 参考磁场：世界系下磁北沿 +X，水平分量保留 */
    float bx = sqrtf(hx * hx + hy * hy);

    /* 参考方向再转回机体系：w = R^T * (bx, 0, bz) */
    float bz = 2.0f * (mx * (q1 * q3 - q0 * q2) + my * (q2 * q3 + q0 * q1) +
                       mz * (0.5f - q1 * q1 - q2 * q2));

    float wx = 2.0f * (bx * (0.5f - q2 * q2 - q3 * q3) + bz * (q1 * q3 - q0 * q2));
    float wy = 2.0f * (bx * (q1 * q2 - q0 * q3) + bz * (q2 * q3 + q0 * q1));
    float wz = 2.0f * (bx * (q1 * q3 + q0 * q2) + bz * (0.5f - q1 * q1 - q2 * q2));

    /* 误差 = 实测磁场 × 参考磁场（体坐标系） */
    *ex = (my * wz - mz * wy) * t;
    *ey = (mz * wx - mx * wz) * t;
    *ez = (mx * wy - my * wx) * t;
}

void ahrs_update_mag(ahrs_t *a, const float gyro[3], const float accel[3],
                     const float mag[3], float dt, float trust_accel, float trust_mag)
{
    float ax_, ay_, az_, mx_, my_, mz_;
    accel_error(a, accel, trust_accel, &ax_, &ay_, &az_);
    mag_error(a, mag, trust_mag, &mx_, &my_, &mz_);
    ahrs_update_core(a, gyro, ax_ + mx_, ay_ + my_, az_ + mz_, dt);
}

void ahrs_get_euler(const ahrs_t *a, float *roll, float *pitch, float *yaw)
{
    const float q0 = a->q[0], q1 = a->q[1], q2 = a->q[2], q3 = a->q[3];

    if (roll) {
        *roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                       1.0f - 2.0f * (q1 * q1 + q2 * q2));
    }
    if (pitch) {
        *pitch = asinf(clampf(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f));
    }
    if (yaw) {
        *yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                      1.0f - 2.0f * (q2 * q2 + q3 * q3));
    }
}

void ahrs_get_euler_deg(const ahrs_t *a, float *roll, float *pitch, float *yaw)
{
    float r, p, y;
    ahrs_get_euler(a, &r, &p, &y);
    if (roll) {
        *roll = r * RAD2DEG;
    }
    if (pitch) {
        *pitch = p * RAD2DEG;
    }
    if (yaw) {
        *yaw = y * RAD2DEG;
    }
}

void ahrs_quat_to_matrix(const float q[4], float m[9])
{
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    m[0] = 1.0f - 2.0f * (y * y + z * z);
    m[1] = 2.0f * (x * y - w * z);
    m[2] = 2.0f * (x * z + w * y);
    m[3] = 2.0f * (x * y + w * z);
    m[4] = 1.0f - 2.0f * (x * x + z * z);
    m[5] = 2.0f * (y * z - w * x);
    m[6] = 2.0f * (x * z - w * y);
    m[7] = 2.0f * (y * z + w * x);
    m[8] = 1.0f - 2.0f * (x * x + y * y);
}

/*
 * 倾斜补偿的磁航向（度）。正方向与 yaw 约定一致（俯视逆时针为正）。
 *
 * 为什么必须"倾斜补偿"：磁力计读到的是三维磁场矢量。板子倾斜时，地球磁场的
 * 竖直分量会混进 X/Y 读数里，直接 atan2(my, mx) 得到的不是航向。
 * 做法是先用加速度计求出 roll/pitch，把磁场旋到水平面再取角度。
 *
 * 这是一次性初始化用的，不参与后续融合。
 */
float ahrs_mag_heading_deg(const float accel[3], const float mag[3], bool *valid)
{
    if (valid) {
        *valid = false;
    }

    float ax = accel[0], ay = accel[1], az = accel[2];
    float an = sqrtf(ax * ax + ay * ay + az * az);
    float mx = mag[0], my = mag[1], mz = mag[2];
    float mn = sqrtf(mx * mx + my * my + mz * mz);
    if (an < 1e-3f || mn < 1e-3f) {
        return 0.0f;
    }
    float ia = 1.0f / an, im = 1.0f / mn;
    ax *= ia;
    ay *= ia;
    az *= ia;
    mx *= im;
    my *= im;
    mz *= im;

    float pitch = asinf(clampf(-ax, -1.0f, 1.0f));
    float roll = atan2f(ay, az);

    /* 把磁场投影到水平面（机体系）：h = m - (m·g)g，其中 g = -a */
    float gx = -ax, gy = -ay, gz = -az;
    float d = mx * gx + my * gy + mz * gz;
    float hx = mx - d * gx;
    float hy = my - d * gy;
    float hz = mz - d * gz;

    /* 用倾斜角把水平分量转到世界系 */
    float cr = cosf(roll), sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float v1 = cr * hy - sr * hz;
    float v2 = sr * hy + cr * hz;
    float tx = cp * hx + sp * v2;
    float ty = v1;

    if (fabsf(tx) < 1e-6f && fabsf(ty) < 1e-6f) {
        return 0.0f;
    }
    if (valid) {
        *valid = true;
    }
    /* 世界系磁北沿 +X，航向正方向与 yaw 约定一致（逆时针为正） */
    return atan2f(-ty, tx) * RAD2DEG;
}

bool ahrs_init_from_accel_mag(ahrs_t *a, const float accel[3], const float mag[3])
{
    float ax = accel[0], ay = accel[1], az = accel[2];
    float an = sqrtf(ax * ax + ay * ay + az * az);
    if (an < 1e-3f) {
        return false;
    }
    float ia = 1.0f / an;
    ax *= ia;
    ay *= ia;
    az *= ia;

    float pitch = asinf(clampf(-ax, -1.0f, 1.0f));
    float roll = atan2f(ay, az);

    /*
     * 有磁力计就把 yaw 初始化成真实磁航向，而不是 0。
     *
     * 这是踩过的坑：早期版本这里写死 yaw=0（当时做的是 6 轴），接上磁力计后
     * 滤波要花好几秒从 0° 爬到真实航向（实测 6 秒爬了 46°），那段时间屏幕上
     * 的 yaw 是"假漂移"，看起来像磁力计没用。正确做法是一次性算出航向再起滤波。
     */
    float yaw = 0.0f;
    if (mag) {
        bool ok = false;
        float hdg_deg = ahrs_mag_heading_deg(accel, mag, &ok);
        if (ok) {
            yaw = hdg_deg * DEG2RAD; /* 度 -> 弧度，下面四元数用的是弧度 */
        }
    }

    /* ZYX 欧拉角 -> 四元数（R = Rz(yaw) * Ry(pitch) * Rx(roll)） */
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);

    a->q[0] = cr * cp * cy + sr * sp * sy;
    a->q[1] = sr * cp * cy - cr * sp * sy;
    a->q[2] = cr * sp * cy + sr * cp * sy;
    a->q[3] = cr * cp * sy - sr * sp * cy;
    a->integral[0] = a->integral[1] = a->integral[2] = 0.0f;
    quat_normalize(a->q);
    return true;
}
