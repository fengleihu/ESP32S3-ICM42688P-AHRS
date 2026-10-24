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
工具不是按 MARK 硬切段来判定的（那样"标记点之后的那段"必然混进动作过程），
而是先从数据里找出所有**稳定平台**（姿态变化率足够小、持续 ≥2.5 s 的区间），
用平台均值比较，这样标记点不需要掐得准。

  1. 静态：平台上的 yaw σ（去趋势残差）、yaw 漂移（°/min）、|yaw − 磁航向|
  2. 倾斜：两个相邻平台倾斜差 20~45° 时，yaw 变化应 < 2°
  3. 180°：相邻平台 |Δyaw| 在 180 ± 15° 内时，偏差应 ≤ 5°
  4. 表格里的 |B| 与 |B| 起伏是旁证（平台内板子不动，起伏主要反映噪声，
     不单独判 PASS/FAIL）

退出码：0 = 全部可判定的项通过；1 = 有失败项；2 = 没有足够的平台可判定。
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import statistics
import sys

# ATT 列（0 基）
C_MS = 1
C_ROLL, C_PITCH, C_YAW = 2, 3, 4
C_MAG_PRESENT, C_MAG_CAL = 5, 6
C_AX, C_AY, C_AZ = 7, 8, 9
C_FRESH, C_HDG_VALID, C_HDG = 10, 11, 12
C_CAL_MX, C_CAL_MY, C_CAL_MZ = 13, 14, 15
C_RAW_MX, C_RAW_MY, C_RAW_MZ = 16, 17, 18

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
    """返回 (rows, marks, highlights)。

    rows: dict 列表（ms/t/roll/pitch/yaw/...）
    marks: (t_seconds, label) 列表，按时间排序
    highlights: 含关键字的固件日志行
    """
    rows, marks, highlights = [], [], []
    t0 = None
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
                row["t"] = (ms - t0) / 1000.0
                rows.append(row)
            elif line.startswith("MARK,"):
                parts = line.split(",", 2)
                if len(parts) >= 2:
                    try:
                        ms = int(parts[1])
                    except ValueError:
                        continue
                    label = parts[2].strip() if len(parts) > 2 else "mark"
                    marks.append(((ms - t0) / 1000.0 if t0 is not None else 0.0,
                                  label))
            elif any(k in line for k in LOG_HIGHLIGHTS):
                highlights.append(line)
    marks.sort(key=lambda m: m[0])
    return rows, marks, highlights


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
    diffs = [wrap180(r[C_YAW] - r[C_HDG]) for r in sub if r[C_HDG_VALID] > 0.5]
    return {
        "t0": ts[0],
        "t1": ts[-1],
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


def analyze_file(path, args):
    rows, marks, highlights = parse_log(path)
    print("=" * 82)
    print(f"文件: {path}")
    if not rows:
        print("  没有 ATT 行（是不是抓的是启动日志？）")
        return None

    total = rows[-1]["t"] - rows[0]["t"]
    print(f"  ATT {len(rows)} 行，{total:.1f} s（{len(rows) / max(total, 1e-3):.1f} Hz），"
          f"MARK {len(marks)} 个")
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

    stats = []
    for a, b in plateaus:
        st = plateau_stats(rows, a, b)
        st["label"] = label_for(marks, st["t0"], st["t1"])
        stats.append(st)

    print(f"\n== 稳定平台（变化率 ≤{STEADY_RATE_DPS}°/s，长度 ≥{MIN_PLATEAU_S}s）==")
    head = ("#", "标签", "起点s", "时长s", "roll", "pitch", "yaw", "倾斜°",
            "σ°", "漂移°/min", "yaw−磁航向°", "|B|µT", "|B|起伏%")
    print(("{:<3}{:<13}{:>8}" + "{:>9}" * 10).format(*head))
    for k, st in enumerate(stats):
        print(("{:<3}{:<13}{:>8}" + "{:>9}" * 10).format(
            k, st["label"][:12], fmt(st["t0"], 1), fmt(st["dur"], 1),
            fmt(st["roll"], 1), fmt(st["pitch"], 1), fmt(st["yaw"], 1),
            fmt(st["tilt"], 1), fmt(st["sigma"], 3), fmt(st["drift"], 2, True),
            fmt(st["yaw_minus_hdg"], 2, True), fmt(st["bmean"], 1),
            fmt(st["bscale"], 1)))

    checks = []

    # ---- 静态判据：取各平台上最差的（只算足够的时长）----
    static = [st for st in stats if st["dur"] >= 5.0 and st["mag_frac"] > 0.5]
    if static:
        worst_sigma = max(static, key=lambda s: s["sigma"])
        worst_hdg = max(static, key=lambda s: abs(s["yaw_minus_hdg"])
                        if not math.isnan(s["yaw_minus_hdg"]) else 0.0)
        checks.append(("静态 yaw σ", worst_sigma["sigma"], args.max_sigma, "°",
                       worst_sigma["label"]))
        if not math.isnan(worst_hdg["yaw_minus_hdg"]):
            checks.append(("|yaw−磁航向|", abs(worst_hdg["yaw_minus_hdg"]),
                           args.max_hdg, "°", worst_hdg["label"]))
        long_static = [st for st in static if st["dur"] >= 20.0]
        if long_static:
            worst_drift = max(long_static, key=lambda s: abs(s["drift"]))
            checks.append(("yaw 漂移", abs(worst_drift["drift"]), args.max_drift,
                           "°/min", worst_drift["label"]))

    # ---- 相邻平台之间的变化 ----
    if len(stats) >= 2:
        print("\n== 相邻平台变化 ==")
        print("{:<16}->{:<16}{:>9}{:>9}{:>9}{:>9}".format(
            "从", "到", "Δroll", "Δpitch", "Δyaw", "Δ倾斜"))
        pairs = []
        for i in range(len(stats) - 1):
            a, b = stats[i], stats[i + 1]
            p = {
                "a": a["label"], "b": b["label"],
                "d_roll": b["roll"] - a["roll"],
                "d_pitch": b["pitch"] - a["pitch"],
                "d_yaw": wrap180(b["yaw"] - a["yaw"]),
                "d_tilt": b["tilt"] - a["tilt"],
            }
            pairs.append(p)
            print("{:<16}->{:<16}{:>+9.1f}{:>+9.1f}{:>+9.1f}{:>+9.1f}".format(
                p["a"][:15], p["b"][:15], p["d_roll"], p["d_pitch"], p["d_yaw"],
                p["d_tilt"]))
        print("  （符号约定与固件一致；Δyaw 已做 ±180° 解缠绕）")

        tilt_pairs = [p for p in pairs if 20.0 <= abs(p["d_tilt"]) <= 45.0]
        if tilt_pairs:
            worst = max(tilt_pairs, key=lambda p: abs(p["d_yaw"]))
            checks.append((f"倾斜 {abs(worst['d_tilt']):.0f}° 时 yaw 变化",
                           abs(worst["d_yaw"]), args.max_tilt_yaw, "°",
                           f"{worst['a']}->{worst['b']}"))
        turn_pairs = [p for p in pairs
                      if abs(p["d_tilt"]) <= 8.0 and
                      abs(abs(p["d_yaw"]) - 180.0) <= 15.0]
        if turn_pairs:
            worst = max(turn_pairs, key=lambda p: abs(abs(p["d_yaw"]) - 180.0))
            checks.append((f"180° 转身偏差（实测 {worst['d_yaw']:+.1f}°）",
                           abs(abs(worst["d_yaw"]) - 180.0), args.max_turn_err, "°",
                           f"{worst['a']}->{worst['b']}"))

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
    ana.add_argument("--max-sigma", type=float, default=0.5)
    ana.add_argument("--max-drift", type=float, default=2.0)
    ana.add_argument("--max-hdg", type=float, default=1.0)
    ana.add_argument("--max-tilt-yaw", type=float, default=2.0)
    ana.add_argument("--max-turn-err", type=float, default=5.0)

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
