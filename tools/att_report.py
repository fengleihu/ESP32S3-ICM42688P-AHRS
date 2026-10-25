#!/usr/bin/env python3
"""att_report.py — 采集并判定 ESP32-S3 AHRS 的四项静态/动态判据。

固件每 100 ms 输出一行 `ATT,...`（19 列），并接受两条件串口命令：
  - `RECAL_MAG`：清掉磁校准并重启，进 15 秒的校准流程；
  - `MARK <label>`：立刻输出一行 `MARK,<ms>,<label>`，给测量点命名。

用法
----
记录一段串口日志（需要 pyserial；记录时可以直接在终端打字，回车即发给设备）：

    python tools/att_report.py record --port COM6 --seconds 90 --out logs/session.log

推荐的测量流程（每次动作后打一个点，位置随意，不必掐表）：

    MARK level       # 板子水平放好
    MARK tilt30      # 倾到 30° 左右并稳住
    MARK level2      # 回水平
    MARK turn180     # 原地（绕竖直轴）转 180° 并稳住

分析日志（只读文件，不需要硬件）：

    python tools/att_report.py analyze logs/session.log

判定方式
--------
优先按 `MARK` 判：把每个 MARK 之前的 3 秒当成该状态的测量窗口（人是在摆好之后
才按的按钮，标记即状态末尾），再比较相邻状态的差值。没有 MARK 时才退回"从数据里
自动找稳定平台"的做法；静态 σ/漂移 始终用自动识别的长静止段（短窗口估不出漂移）。

工具不是按时间切割数据的（前几轮靠"45 秒后开始倾斜"全废了），测量的时间点完全
由按钮/MARK 决定。

  1. 静态：平台上的 yaw σ（去趋势残差）、yaw 漂移（°/min）、|yaw − 磁航向|
  2. 倾斜：两个相邻平台倾斜差 20~45° 时，yaw 变化应 < 2°
  3. 180°：相邻平台 |Δyaw| 在 180 ± 15° 内时，偏差应 ≤ 5°
  4. 磁一致性：同一段里"磁航向变化"和"陀螺 Z 轴积分"应一致（默认差 ≤2°）。
     差得多说明板子其实没转，是磁侧在变 —— 磁畸变或倾斜补偿误差。这一项需要
     固件输出 ATT 的第 19~21 列（三轴角速度）；老固件的日志会自动跳过。
  5. 表格里的 |B| 与 |B| 起伏是旁证（平台内板子不动，起伏主要反映噪声，
     不单独判 PASS/FAIL）；但跨平台 |B| 相差 >2% 会提示测过程中磁场环境变了。

退出码：0 = 全部可判定的项通过；1 = 有失败项；2 = 没有足够的平台可判定。
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import statistics
import sys
from bisect import bisect_left, bisect_right

# ATT 列（0 基）
C_MS = 1
C_ROLL, C_PITCH, C_YAW = 2, 3, 4
C_MAG_PRESENT, C_MAG_CAL = 5, 6
C_AX, C_AY, C_AZ = 7, 8, 9
C_FRESH, C_HDG_VALID, C_HDG = 10, 11, 12
C_CAL_MX, C_CAL_MY, C_CAL_MZ = 13, 14, 15
C_RAW_MX, C_RAW_MY, C_RAW_MZ = 16, 17, 18
C_GX, C_GY, C_GZ = 19, 20, 21  # 可选：固件新增的三轴角速度（°/s）

# 值得回填 HANDOFF 的固件日志行
LOG_HIGHLIGHTS = (
    "陀螺零偏",
    "ICM42688P",
    "BMM350 在线",
    "已载入磁力计校准",
    "已载入旧版",
    "校准完成",
    "Mag calibration OK",
    "轴向映射",
    "Serial command enabled",
)

# 平台判定阈值：±0.6 s 窗口内的最小二乘斜率
HALF_WINDOW_S = 0.6
STEADY_RATE_DPS = 6.0
MIN_PLATEAU_S = 2.5


# --------------------------------------------------------------------------
# 角度工具
# --------------------------------------------------------------------------
def wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def circ_mean_deg(vals) -> float:
    """圆周均值（度）。yaw/航向跨 ±180 时不能直接取算术平均。"""
    if not vals:
        return 0.0
    s = sum(math.sin(math.radians(v)) for v in vals)
    c = sum(math.cos(math.radians(v)) for v in vals)
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        return 0.0
    return math.degrees(math.atan2(s, c))


def unwrap_deg(vals):
    """把 ±180 跳变解掉，得到连续的角度序列。"""
    out = []
    offset = 0.0
    prev = None
    for v in vals:
        if prev is not None:
            d = wrap180(v - prev)
            offset += d - (v - prev)
        out.append(v + offset)
        prev = v
    return out


def linfit(xs, ys):
    """最小二乘直线 y = a*x + b，返回 (a, b, 残差 RMS)。"""
    n = len(xs)
    if n < 2:
        return 0.0, ys[0] if ys else 0.0, 0.0
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    a = sxy / sxx if sxx > 1e-12 else 0.0
    b = my - a * mx
    res = [y - (a * x + b) for x, y in zip(xs, ys)]
    rms = math.sqrt(sum(r * r for r in res) / n)
    return a, b, rms


# --------------------------------------------------------------------------
# 日志解析
# --------------------------------------------------------------------------
def parse_log(path):
    """返回 (rows, marks, highlights, reboot_ts)。

    rows: dict 列表（ms/t/roll/pitch/yaw/...）
    marks: (t_seconds, label) 列表，按时间排序
    highlights: 含关键字的固件日志行
    reboot_ts: 检测到的重启时刻（t，秒）

    ATT 的时间列是 esp_timer 的毫秒数，**重启后会归零**。日志里跨一次重启
    （比如 RECAL_MAG 那次）就会出现时间倒退，稳定平台检测会直接失效
    （第一版实测输出 -83.0 s）。这里给重启后的时间加一个偏移，让它继续
    单调递增；偏移里补 1.5 s 作为重启耗时，只影响绝对时刻，不影响段内分析。
    """
    rows, marks, highlights = [], [], []
    t0 = None
    prev_ms = None
    offset_ms = 0
    reboot_ts = []
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            line = raw.rstrip("\r\n")
            if line.startswith("ATT,"):
                parts = line.split(",")
                if len(parts) < 19:
                    continue
                try:
                    ms = int(parts[C_MS])
                    row = {"ms": ms}
                    for idx in range(C_ROLL, len(parts)):
                        row[idx] = float(parts[idx])
                except ValueError:
                    continue
                if t0 is None:
                    t0 = ms
                if prev_ms is not None and ms < prev_ms:
                    offset_ms += prev_ms - ms + 1500
                    reboot_ts.append((ms + offset_ms - t0) / 1000.0)
                prev_ms = ms
                row["t"] = (ms + offset_ms - t0) / 1000.0
                rows.append(row)
            elif line.startswith("MARK,"):
                parts = line.split(",", 2)
                if len(parts) >= 2:
                    try:
                        ms = int(parts[1])
                    except ValueError:
                        continue
                    label = parts[2].strip() if len(parts) > 2 else "mark"
                    marks.append(((ms + offset_ms - t0) / 1000.0
                                  if t0 is not None else 0.0, label))
            elif any(k in line for k in LOG_HIGHLIGHTS):
                highlights.append(line)
    marks.sort(key=lambda m: m[0])
    return rows, marks, highlights, reboot_ts


def tilt_deg(row) -> float:
    """机体 +Z 与重力反方向的夹角（度）。水平时 ≈ 0。"""
    ax, ay, az = row[C_AX], row[C_AY], row[C_AZ]
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-6:
        return 0.0
    c = max(-1.0, min(1.0, az / n))
    return math.degrees(math.acos(c))


def bmag(row) -> float:
    return math.sqrt(row[C_CAL_MX] ** 2 + row[C_CAL_MY] ** 2 + row[C_CAL_MZ] ** 2)


def has_gyro(rows) -> bool:
    return bool(rows) and C_GZ in rows[0]


def cum_gyro_z(rows):
    """陀螺 Z 轴的累计角度（度），用来给航向变化做独立参考。

    为什么需要：只看 yaw/磁航向分不清"人真的把板子转了"和"磁航向被倾斜
    补偿算歪了"。陀螺在这几秒内的积分是独立于磁的：两者对不上，问题就在磁侧。
    """
    cum = [0.0] * len(rows)
    for i in range(1, len(rows)):
        dt = rows[i]["t"] - rows[i - 1]["t"]
        cum[i] = cum[i - 1] + 0.5 * (rows[i][C_GZ] + rows[i - 1][C_GZ]) * dt
    return cum


# --------------------------------------------------------------------------
# 稳定平台
# --------------------------------------------------------------------------
def find_plateaus(rows):
    """找出姿态基本不动的区间，返回 [(i_start, i_end)]（含端点）。

    判据是 ±0.6 s 窗口内的最小二乘斜率，而不是"两点差分"：yaw 本身有 0.1~1.5°
    的噪声，两点差分会把噪声变成几 °/s 的假运动，于是好的数据也被判成"一直在动"。
    拟合斜率对同样窗口的噪声不敏感（噪声 σ 1.6° 时斜率误差 ≈1.3°/s）。
    """
    n = len(rows)
    if n < 9:
        return []
    ts = [r["t"] for r in rows]
    tilts = [tilt_deg(r) for r in rows]
    yaws = unwrap_deg([r[C_YAW] for r in rows])

    def prefix(vals):
        out = [0.0]
        for v in vals:
            out.append(out[-1] + v)
        return out

    p_t = prefix(ts)
    p_tt = prefix([t * t for t in ts])
    p_tilt = prefix(tilts)
    p_ttilt = prefix([t * v for t, v in zip(ts, tilts)])
    p_yaw = prefix(yaws)
    p_tyaw = prefix([t * v for t, v in zip(ts, yaws)])

    def slope(pv, ptv, lo, hi):
        cnt = hi - lo + 1
        st = p_t[hi + 1] - p_t[lo]
        stt = p_tt[hi + 1] - p_tt[lo]
        sv = pv[hi + 1] - pv[lo]
        stv = ptv[hi + 1] - ptv[lo]
        den = cnt * stt - st * st
        if abs(den) < 1e-12:
            return 0.0
        return (cnt * stv - st * sv) / den

    steady = [False] * n
    lo = 0
    hi = 0
    for i in range(n):
        while ts[i] - ts[lo] > HALF_WINDOW_S:
            lo += 1
        if hi < i:
            hi = i
        while hi + 1 < n and ts[hi + 1] - ts[i] <= HALF_WINDOW_S:
            hi += 1
        if ts[i] - ts[lo] < 0.3 or ts[hi] - ts[i] < 0.3:
            continue
        dtilt = abs(slope(p_tilt, p_ttilt, lo, hi))
        dyaw = abs(slope(p_yaw, p_tyaw, lo, hi))
        steady[i] = dtilt <= STEADY_RATE_DPS and dyaw <= STEADY_RATE_DPS

    runs = []
    start = None
    for i, ok in enumerate(steady):
        if ok and start is None:
            start = i
        elif not ok and start is not None:
            runs.append((start, i - 1))
            start = None
    if start is not None:
        runs.append((start, n - 1))

    return [(a, b) for a, b in runs if ts[b] - ts[a] >= MIN_PLATEAU_S]


def world_field(row):
    """机体坐标系磁场旋到世界系（只用加速度计的 roll/pitch）。

    磁倾角 dip 是**与姿态无关的不变量**：板子在同一地点无论怎么倾斜，dip 都
    应该一样。所以 dip 随倾斜变化就是"轴映射/软磁/倾斜补偿"有问题的直接证据，
    而且它不需要知道板子的真实朝向 —— 比"倾斜 30° 看 yaw 变多少"更抗干扰。
    """
    ax, ay, az = row[C_AX], row[C_AY], row[C_AZ]
    n = math.sqrt(ax * ax + ay * ay + az * az)
    if n < 1e-6:
        return None
    ax, ay, az = ax / n, ay / n, az / n
    pitch = math.asin(max(-1.0, min(1.0, -ax)))
    roll = math.atan2(ay, az)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    x, y, z = row[C_CAL_MX], row[C_CAL_MY], row[C_CAL_MZ]
    y1 = cr * y - sr * z
    z1 = sr * y + cr * z
    return (cp * x + sp * z1, y1, -sp * x + cp * z1)


def dip_deg(row):
    w = world_field(row)
    if not w:
        return float("nan")
    return math.degrees(math.atan2(w[2], math.hypot(w[0], w[1])))


def plateau_stats(rows, i0, i1):
    sub = rows[i0:i1 + 1]
    ts = [r["t"] for r in sub]
    yaws = [r[C_YAW] for r in sub]
    unwrapped = unwrap_deg(yaws)
    sigma = 0.0
    drift = 0.0
    if len(sub) >= 5 and ts[-1] - ts[0] >= 1.0:
        slope, _, rms = linfit(ts, unwrapped)
        drift = slope * 60.0
        sigma = rms
    hdgs = [r[C_HDG] for r in sub if r[C_HDG_VALID] > 0.5]
    bs = [bmag(r) for r in sub]
    dips = [dip_deg(r) for r in sub]
    dips = [d for d in dips if not math.isnan(d)]
    diffs = [wrap180(r[C_YAW] - r[C_HDG]) for r in sub if r[C_HDG_VALID] > 0.5]
    return {
        "i0": i0,
        "i1": i1,
        "t0": ts[0],
        "t1": ts[-1],
        "mid": (i0 + i1) // 2,
        "dur": ts[-1] - ts[0],
        "n": len(sub),
        "roll": statistics.fmean(r[C_ROLL] for r in sub),
        "pitch": statistics.fmean(r[C_PITCH] for r in sub),
        "yaw": circ_mean_deg(yaws),
        "tilt": statistics.fmean(tilt_deg(r) for r in sub),
        "sigma": sigma,
        "drift": drift,
        "hdg": circ_mean_deg(hdgs) if hdgs else float("nan"),
        "yaw_minus_hdg": statistics.fmean(diffs) if diffs else float("nan"),
        "bmean": statistics.fmean(bs),
        "dip": statistics.fmean(dips) if dips else float("nan"),
        "bscale": statistics.pstdev(bs) / statistics.fmean(bs) * 100.0
        if statistics.fmean(bs) > 1e-6 else 0.0,
        "mag_frac": sum(1 for r in sub if r[C_MAG_CAL] > 0.5) / len(sub),
    }


def label_for(marks, t0, t1):
    """给平台找标签：取时间上最近的那个 MARK（允许在平台结束之后）。

    用户通常是在"摆好状态之后"打点，所以标签往往贴着平台的尾巴。取最近的
    那个点比"取平台开始前的最后一个"更符合直觉。
    """
    best = None
    for mt, ml in marks:
        dist = 0.0
        if mt < t0:
            dist = t0 - mt
        elif mt > t1:
            dist = mt - t1
        if best is None or dist < best[0]:
            best = (dist, ml)
    if best is None or best[0] > 10.0:
        return "<start>"
    return best[1]


# --------------------------------------------------------------------------
# 输出
# --------------------------------------------------------------------------
def fmt(x, nd=2, sign=False):
    if x is None or (isinstance(x, float) and math.isnan(x)):
        return "n/a"
    return f"{x:+.{nd}f}" if sign else f"{x:.{nd}f}"


def trim_settle(rows, i0, i1, windows):
    """把平台开头落在"重启后的静默窗口"里的部分剪掉。

    为什么需要：RECAL_MAG 之后板子会重启并重新做初始对准，而人往往还在
    扶着板子；重启后头 20~30 s 的 yaw 会明显爬行（实测 3.4° 在 15 s 内），
    这段算进 σ 会让一个校准良好的系统看起来完全不达标（实测 σ 0.53° 对
    0.5° 门限）。窗口长度用 --settle 调，默认 30 s。
    """
    moved = True
    while moved:
        moved = False
        for s0, s1 in windows:
            if rows[i1]["t"] <= s0 or rows[i0]["t"] >= s1:
                continue
            j = i0
            while j <= i1 and rows[j]["t"] < s1:
                j += 1
            if j > i0:
                i0 = j
                moved = True
    return i0, i1


def windows_from_marks(rows, marks, window_s):
    """把每个 MARK 之前的 window_s 秒当成该状态的测量窗口。

    为什么不从 MARK 往后取：人是在"摆好并稳住之后"按下按钮的，标记时刻就是
    该状态的末尾；标记之后的区间是"开始动"的过程。所以取前不取后。
    """
    out = []
    ts = [r["t"] for r in rows]
    for t_mark, label in marks:
        hi = bisect_right(ts, t_mark) - 1
        lo = bisect_left(ts, t_mark - window_s)
        if hi - lo < 5 or ts[hi] - ts[lo] < 1.0:
            continue
        st = plateau_stats(rows, lo, hi)
        st["label"] = label
        out.append(st)
    return out


def print_stats_table(title, stats, dip_med):
    print(f"\n{title}")
    head = ("#", "标签", "起点s", "时长s", "roll", "pitch", "yaw", "倾斜°",
            "σ°", "漂移°/min", "yaw−磁航向°", "dip°", "|B|µT", "|B|起伏%")
    print(("{:<3}{:<13}{:>8}" + "{:>9}" * 11).format(*head))
    for k, st in enumerate(stats):
        print(("{:<3}{:<13}{:>8}" + "{:>9}" * 11).format(
            k, (st["label"] + ("!" if st.get("env_out") else ""))[:12],
            fmt(st["t0"], 1), fmt(st["dur"], 1), fmt(st["roll"], 1),
            fmt(st["pitch"], 1), fmt(st["yaw"], 1), fmt(st["tilt"], 1),
            fmt(st["sigma"], 3), fmt(st["drift"], 2, True),
            fmt(st["yaw_minus_hdg"], 2, True), fmt(st["dip"], 1),
            fmt(st["bmean"], 1), fmt(st["bscale"], 1)))
    if dip_med is not None and not math.isnan(dip_med):
        print(f"  dip 是中纬度磁场的不变量，同一地点在不同倾斜下应保持一致；"
              f"中位数 {dip_med:.1f}°")


def analyze_file(path, args):
    rows, marks, highlights, reboot_ts = parse_log(path)
    print("=" * 82)
    print(f"文件: {path}")
    if not rows:
        print("  没有 ATT 行（是不是抓的是启动日志？）")
        return None

    total = rows[-1]["t"] - rows[0]["t"]
    print(f"  ATT {len(rows)} 行，{total:.1f} s（{len(rows) / max(total, 1e-3):.1f} Hz），"
          f"MARK {len(marks)} 个，重启 {len(reboot_ts)} 次")
    if marks:
        print("  标记: " + "  ".join(f"{t:.1f}s={l}" for t, l in marks))
    mag_frac = sum(1 for r in rows if r[C_MAG_CAL] > 0.5) / len(rows)
    print(f"  9 轴模式占比 {mag_frac * 100:.0f}%")
    if highlights:
        print("  日志要点:")
        for h in highlights[:10]:
            print(f"    {h}")

    plateaus = find_plateaus(rows)
    if not plateaus:
        print("  没有找到稳定平台：整段都在动，或者日志太短（<2.5 s）")
        return None

    windows = [(rt, rt + args.settle) for rt in reboot_ts]
    if windows:
        print(f"  重启后的前 {args.settle:.0f} s 视为滤波收敛期，不计入静态指标")

    plateau_list = []
    skipped = 0
    for a, b in plateaus:
        a2, b2 = trim_settle(rows, a, b, windows)
        if b2 <= a2 or rows[b2]["t"] - rows[a2]["t"] < MIN_PLATEAU_S:
            skipped += 1
            continue
        st = plateau_stats(rows, a2, b2)
        st["label"] = label_for(marks, st["t0"], st["t1"])
        plateau_list.append(st)

    if skipped:
        print(f"  有 {skipped} 个平台整段落在收敛窗口内，已跳过")

    # 有 MARK 就用"每个 MARK 之前 state_window 秒"当该状态的窗口（人是在摆好之后
    # 才按的按钮）；没有 MARK 才退回自动平台识别。
    state_list = windows_from_marks(rows, marks, args.state_window) if marks else []
    if state_list:
        print(f"  按 {len(marks)} 个 MARK 取窗口：每个 MARK 之前 "
              f"{args.state_window:.1f} s（标记即状态末尾）")
    else:
        state_list = plateau_list
    if not state_list:
        print("  没有可用的数据窗口")
        return None

    # ---- 环境一致性：|B| 或 dip 与中位数差太多，说明这段数据搬了位置/被遮挡 ----
    ref = plateau_list or state_list
    bvals = [st["bmean"] for st in ref if st["bmean"] > 1e-6]
    dipvals = [st["dip"] for st in ref if not math.isnan(st["dip"])]
    b_med = statistics.median(bvals) if bvals else 0.0
    dip_med = statistics.median(dipvals) if dipvals else 0.0
    for st in state_list + plateau_list:
        st["env_out"] = False
        if b_med > 1e-6 and abs(st["bmean"] - b_med) / b_med > args.max_b_drift:
            st["env_out"] = True
        if dipvals and abs(st["dip"] - dip_med) > args.max_dip_drift:
            st["env_out"] = True

    print_stats_table("== 测量状态 ==", state_list, dip_med)
    if marks and plateau_list:
        long_plateaus = [st for st in plateau_list if st["dur"] >= 10.0]
        if long_plateaus:
            print_stats_table("== 自动识别的长时间静止段（用于 σ / 漂移） ==",
                              long_plateaus, dip_med)

    env_bad = [st for st in state_list if st["env_out"]]
    if env_bad:
        print(f"  ! 带 ! 的 {len(env_bad)} 个状态窗口 |B| 或 dip 偏离中位数"
              f"（> {args.max_b_drift * 100:.0f}% / {args.max_dip_drift:.1f}°）："
              f"该段磁场环境与其它段不同（换位置、靠近金属或被手遮挡），"
              f"不参与判据")

    checks = []

    # ---- 静态判据：优先用长静止段（至少 20 s 才能谈漂移）----
    static_pool = [st for st in (plateau_list or state_list)
                   if st["dur"] >= 5.0 and st["mag_frac"] > 0.5 and not st["env_out"]]
    if static_pool:
        worst_sigma = max(static_pool, key=lambda s: s["sigma"])
        worst_hdg = max(static_pool, key=lambda s: abs(s["yaw_minus_hdg"])
                        if not math.isnan(s["yaw_minus_hdg"]) else 0.0)
        checks.append(("静态 yaw σ", worst_sigma["sigma"], args.max_sigma, "°",
                       worst_sigma["label"]))
        if not math.isnan(worst_hdg["yaw_minus_hdg"]):
            checks.append(("|yaw−磁航向|", abs(worst_hdg["yaw_minus_hdg"]),
                           args.max_hdg, "°", worst_hdg["label"]))
        long_static = [st for st in static_pool if st["dur"] >= 20.0]
        if long_static:
            worst_drift = max(long_static, key=lambda s: abs(s["drift"]))
            checks.append(("yaw 漂移", abs(worst_drift["drift"]), args.max_drift,
                           "°/min", worst_drift["label"]))

    # ---- 相邻状态之间的变化 ----
    if len(state_list) >= 2:
        gyro = has_gyro(rows)
        cum = cum_gyro_z(rows) if gyro else None
        print("\n== 相邻状态变化 ==")
        if gyro:
            print("{:<14}->{:<14}{:>8}{:>8}{:>8}{:>8}{:>10}".format(
                "从", "到", "Δroll", "Δpitch", "Δyaw", "Δ倾斜", "Δyaw陀螺"))
        else:
            print("{:<16}->{:<16}{:>9}{:>9}{:>9}{:>9}".format(
                "从", "到", "Δroll", "Δpitch", "Δyaw", "Δ倾斜"))
        pairs = []
        for i in range(len(state_list) - 1):
            a, b = state_list[i], state_list[i + 1]
            p = {
                "a": a["label"], "b": b["label"],
                "d_roll": b["roll"] - a["roll"],
                "d_pitch": b["pitch"] - a["pitch"],
                "d_yaw": wrap180(b["yaw"] - a["yaw"]),
                "d_tilt": b["tilt"] - a["tilt"],
                "usable": not (a["env_out"] or b["env_out"]),
                # 只在"上一段结束 → 下一段开始"这段运动区间里积分陀螺：
                # 窗口越短，残余零偏（0.01~0.02°/s）累积得越少
                "d_yaw_gyro": (cum[b["i0"]] - cum[a["i1"]]) if gyro else None,
            }
            pairs.append(p)
            note = "" if p["usable"] else "   （环境不一致，跳过）"
            if gyro:
                print("{:<14}->{:<14}{:>+8.1f}{:>+8.1f}{:>+8.1f}{:>+8.1f}{:>+10.1f}"
                      .format(p["a"][:13], p["b"][:13], p["d_roll"],
                              p["d_pitch"], p["d_yaw"], p["d_tilt"],
                              p["d_yaw_gyro"]) + note)
            else:
                print("{:<16}->{:<16}{:>+9.1f}{:>+9.1f}{:>+9.1f}{:>+9.1f}".format(
                    p["a"][:15], p["b"][:15], p["d_roll"], p["d_pitch"],
                    p["d_yaw"], p["d_tilt"]) + note)
        print("  （Δyaw 已做 ±180° 解缠绕；Δyaw陀螺 = 两段之间陀螺 Z 轴积分，"
              "独立于磁，用来扣掉人手实际转过的角度）")

        if gyro:
            for p in pairs:
                if not p["usable"] or p["d_yaw_gyro"] is None:
                    continue
                gap = wrap180(p["d_yaw"] - p["d_yaw_gyro"])
                if abs(gap) > args.max_mag_gap:
                    print(f"  ! {p['a']}->{p['b']}: 磁航向变化 {p['d_yaw']:+.1f}° vs "
                          f"陀螺积分 {p['d_yaw_gyro']:+.1f}°，差 {gap:+.1f}° —— "
                          f"板子实际只转了 {p['d_yaw_gyro']:+.1f}°，多出来的部分是"
                          f"磁侧（畸变或倾斜补偿误差）")
            usable_gaps = [abs(wrap180(p["d_yaw"] - p["d_yaw_gyro"]))
                           for p in pairs if p["usable"] and p["d_yaw_gyro"] is not None]
            if usable_gaps:
                checks.append(("磁航向 vs 陀螺积分的最大偏差", max(usable_gaps),
                               args.max_mag_gap, "°", "任一对可用平台"))

        tilt_pairs = [p for p in pairs if p["usable"]
                      and 20.0 <= abs(p["d_tilt"]) <= 45.0]
        if tilt_pairs:
            if gyro:
                worst = max(tilt_pairs,
                            key=lambda p: abs(wrap180(p["d_yaw"] - p["d_yaw_gyro"])))
                err = abs(wrap180(worst["d_yaw"] - worst["d_yaw_gyro"]))
                checks.append((f"倾斜 {abs(worst['d_tilt']):.0f}° 时磁航向误差"
                               f"（已扣除实际转动 {worst['d_yaw_gyro']:+.1f}°）",
                               err, args.max_tilt_yaw, "°",
                               f"{worst['a']}->{worst['b']}"))
            else:
                worst = max(tilt_pairs, key=lambda p: abs(p["d_yaw"]))
                checks.append((f"倾斜 {abs(worst['d_tilt']):.0f}° 时 yaw 变化",
                               abs(worst["d_yaw"]), args.max_tilt_yaw, "°",
                               f"{worst['a']}->{worst['b']}"))

        turn_pairs = [p for p in pairs if p["usable"]
                      and abs(p["d_tilt"]) <= 8.0
                      and abs(p["d_yaw_gyro"] if gyro else p["d_yaw"]) >= 120.0]
        if turn_pairs:
            if gyro:
                worst = max(turn_pairs,
                            key=lambda p: abs(wrap180(p["d_yaw"] - p["d_yaw_gyro"])))
                err = abs(wrap180(worst["d_yaw"] - worst["d_yaw_gyro"]))
                checks.append((f"转身 {abs(worst['d_yaw_gyro']):.0f}°（陀螺实测）时"
                               f"磁航向误差", err, args.max_turn_err, "°",
                               f"{worst['a']}->{worst['b']}"))
            else:
                err = abs(abs(turn_pairs[0]["d_yaw"]) - 180.0)
                checks.append((f"180° 转身偏差（实测 {turn_pairs[0]['d_yaw']:+.1f}°）",
                               err, args.max_turn_err, "°",
                               f"{turn_pairs[0]['a']}->{turn_pairs[0]['b']}"))

    if not checks:
        print("\n没有可判定的项：需要至少一个 ≥5 s 的静态平台，"
              "以及倾斜/转身造成的平台变化。")
        return {"pass": 0, "fail": 0}

    print("\n== 判据 ==")
    npass = nfail = 0
    for name, value, limit, unit, where in checks:
        ok = value <= limit
        npass += ok
        nfail += (not ok)
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {value:.2f}{unit}"
              f"（限 {limit}{unit}，平台 {where}）")
    return {"pass": npass, "fail": nfail}


def add_analyze_options(parser):
    """判据阈值。CLI 和 GUI（tools/att_gui.py）共用，避免两边默认值漂移。"""
    parser.add_argument("--max-sigma", type=float, default=0.5)
    parser.add_argument("--max-drift", type=float, default=2.0)
    parser.add_argument("--max-hdg", type=float, default=1.0)
    parser.add_argument("--max-tilt-yaw", type=float, default=2.0)
    parser.add_argument("--max-turn-err", type=float, default=5.0)
    parser.add_argument("--max-mag-gap", type=float, default=2.0,
                        help="磁航向变化与陀螺积分的允许偏差（超了说明问题在磁侧）")
    parser.add_argument("--max-b-drift", type=float, default=0.02,
                        help="各平台 |B| 相对中位数允许偏离多少（比例，默认 2%%）")
    parser.add_argument("--max-dip-drift", type=float, default=2.0,
                        help="各平台磁倾角相对中位数允许偏离多少度")
    parser.add_argument("--settle", type=float, default=30.0,
                        help="重启后多少秒内的数据不计入静态指标（滤波收敛期）")
    parser.add_argument("--state-window", type=float, default=3.0,
                        help="每个 MARK 之前取多少秒作为该状态的测量窗口")
    return parser


def default_analyze_args():
    p = argparse.ArgumentParser(add_help=False)
    add_analyze_options(p)
    return p.parse_args([])


def main():
    ap = argparse.ArgumentParser(
        description="采集并判定 ESP32-S3 AHRS 的四项判据",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)

    rec = sub.add_parser("record", help="从串口记录日志（需要 pyserial）")
    rec.add_argument("--port", required=True, help="COM 口，例如 COM6")
    rec.add_argument("--baud", type=int, default=115200)
    rec.add_argument("--seconds", type=float, default=90.0)
    rec.add_argument("--out", required=True)
    rec.add_argument("--send", action="append", default=[],
                     help="开始记录后立刻发送的命令，可重复")

    ana = sub.add_parser("analyze", help="分析已记录的日志")
    ana.add_argument("logs", nargs="+")
    add_analyze_options(ana)

    args = ap.parse_args()
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

    if args.cmd == "record":
        return cmd_record(args)
    return cmd_analyze(args)


def cmd_record(args):
    try:
        import serial  # type: ignore
    except ImportError:
        print("需要 pyserial：python -m pip install pyserial", file=sys.stderr)
        return 2

    import threading

    out_dir = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(out_dir, exist_ok=True)

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    stop = threading.Event()

    def stdin_reader():
        while not stop.is_set():
            try:
                line = sys.stdin.readline()
            except Exception:
                return
            if not line:
                return
            line = line.strip()
            if not line:
                continue
            ser.write((line + "\n").encode())
            print(f"\n--> {line}", flush=True)

    if sys.stdin is not None and sys.stdin.isatty():
        threading.Thread(target=stdin_reader, daemon=True).start()
        print("提示：直接输入 MARK <标签> 并回车，就能给当前状态打点。")

    for cmd in args.send:
        ser.write((cmd + "\n").encode())
        print(f"--> {cmd}")

    n_att = 0
    t_start = None
    print(f"记录 {args.seconds:.0f} s 到 {args.out}（Ctrl+C 提前结束）")
    with open(args.out, "w", encoding="utf-8") as fh:
        import time
        t_end = time.time() + args.seconds
        try:
            while time.time() < t_end:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                fh.write(line + "\n")
                fh.flush()
                if line.startswith("ATT,"):
                    n_att += 1
                    if t_start is None:
                        t_start = time.time()
                elif line.startswith("MARK,") or "校准" in line:
                    print(line)
        except KeyboardInterrupt:
            print("\n提前结束")
    stop.set()
    ser.close()
    dur = max(time.time() - (t_start or time.time()), 1e-3)
    print(f"完成：{n_att} 行 ATT（约 {n_att / dur:.1f} Hz）")
    return 0


def cmd_analyze(args):
    paths = []
    for pattern in args.logs:
        matched = glob.glob(pattern)
        paths.extend(matched if matched else [pattern])
    if not paths:
        print("没有匹配的日志文件", file=sys.stderr)
        return 2

    totals = {"pass": 0, "fail": 0}
    for path in paths:
        res = analyze_file(path, args)
        if res:
            totals["pass"] += res["pass"]
            totals["fail"] += res["fail"]

    print("=" * 82)
    if totals["pass"] + totals["fail"] == 0:
        print("没有可判定的判据。")
        return 2
    print(f"合计 PASS {totals['pass']}，FAIL {totals['fail']}")
    return 0 if totals["fail"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
