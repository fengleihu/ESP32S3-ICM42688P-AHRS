#!/usr/bin/env python3
"""make_test_log.py — 生成一份"已知答案"的 ATT 日志，用来回归测试 att_report.py。

为什么需要：att_report.py 是判据工具——它把"通过/不通过"直接给到人。判据工具
本身如果没有测试，最坏情况是它对着一批坏数据全报 PASS。这里生成一份数据已知
的日志（静态 σ/漂移、30° 倾斜、180° 转身都是构造出来的），analyze 必须全 PASS；
再生成一份故意超标的（σ 大、倾斜时 yaw 被带偏），analyze 必须报 FAIL。

    python tools/host_test/make_test_log.py --out build/test_good.log
    python tools/host_test/make_test_log.py --out build/test_bad.log --bad
    python tools/att_report.py analyze build/test_good.log   # 退出码 0
    python tools/att_report.py analyze build/test_bad.log    # 退出码 1
"""

import argparse
import math
import random


def gen(path, bad=False, seed=42):
    rnd = random.Random(seed)
    lines = []
    lines.append("I (1200) att9axis: 已载入磁力计校准（椭球拟合）：offset=[24.6, -39.8, 11.9] µT 半径=57.4 µT 质量=0.88 残差=1.20%")
    lines.append("I (1180) icm: ICM42688P ready: addr=0x69 WHO_AM_I=0x47 PWR=0x0F GYRO=0x47 ACCEL=0x67")
    lines.append("I (1100) icm: 陀螺零偏 [-0.39, -0.09, -0.01] deg/s，噪声 0.024 deg/s")

    ms = 1000
    step = 0.1

    # 人真实操作时的顺序：先摆好状态，再打点，然后做下一个动作。
    # 所以动作区间落在"上一个标签之后"，稳定平台检测会把它们排除掉。
    marks = [(2.0, "level"), (50.0, "tilt30"), (70.0, "level2"),
             (90.0, "turn180")]

    drift_rate = 0.2 / 60.0  # °/s
    sigma = 0.08 if not bad else 1.6
    tilt_induced_yaw = 0.15 if not bad else 12.0  # 倾斜时被带偏的角度
    turn_angle = 180.0 if not bad else 174.0

    def emit(t, yaw_true, pitch_true):
        nonlocal ms
        y = yaw_true + rnd.gauss(0.0, sigma)
        p = pitch_true + rnd.gauss(0.0, 0.05)
        r = rnd.gauss(0.0, 0.05)
        hdg = y + 0.02 + rnd.gauss(0.0, 0.05)
        # 加速度：pitch 为正 = 机头向下（固件约定 pitch = asin(-ax)）
        ax = -math.sin(math.radians(p)) * 1000.0
        ay = math.sin(math.radians(r)) * 1000.0
        az = math.cos(math.radians(p)) * math.cos(math.radians(r)) * 1000.0
        b = 57.4 * (1.0 + rnd.gauss(0.0, 0.003))
        mx = b * math.cos(math.radians(hdg)) * math.cos(math.radians(p))
        my = b * math.sin(math.radians(hdg)) * math.cos(math.radians(p))
        mz = -b * math.sin(math.radians(p))
        raw = 1.12
        lines.append(
            "ATT,%d,%.2f,%.2f,%.2f,1,1,%d,%d,%d,1,1,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f"
            % (ms, r, p, y, int(ax), int(ay), int(az), hdg, mx, my, mz,
               raw * mx, raw * my, raw * mz))
        ms += int(step * 1000)

    mark_idx = 0
    t = 0.0
    while t < 95.0:
        while mark_idx < len(marks) and t >= marks[mark_idx][0]:
            lines.append("MARK,%d,%s" % (ms, marks[mark_idx][1]))
            mark_idx += 1

        # 姿态编排
        if t < 30.0:
            pitch_true = 0.0
        elif t < 34.0:
            pitch_true = 30.0 * (t - 30.0) / 4.0
        elif t < 50.0:
            pitch_true = 30.0
        elif t < 54.0:
            pitch_true = 30.0 * (1.0 - (t - 50.0) / 4.0)
        else:
            pitch_true = 0.0

        if t < 70.0:
            yaw_true = drift_rate * t
        elif t < 73.0:
            yaw_true = drift_rate * t + turn_angle * (t - 70.0) / 3.0
        else:
            yaw_true = drift_rate * t + turn_angle
        yaw_true += tilt_induced_yaw * (pitch_true / 30.0)

        emit(t, yaw_true, pitch_true)
        t += step

    lines.append("I (99999) att9axis: Mag calibration OK: samples=375 method=椭球拟合 quality=0.88 radius=57.4 uT residual=1.20%")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    print("写出 %s（%d 行）" % (path, len(lines)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--bad", action="store_true",
                    help="生成超标数据（σ 1.6°、倾斜带偏 12°），analyze 应报 FAIL")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()
    gen(args.out, bad=args.bad, seed=args.seed)


if __name__ == "__main__":
    main()
