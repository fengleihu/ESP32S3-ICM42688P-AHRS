#!/usr/bin/env python3
"""六面标定的算法回归测试（不需要板子、不需要串口）。

tools/att_gui.py 里"采集六面 → 算 bias/scale → 发给固件"这段逻辑：
- 用一组已知的真值（bias 20/-15/8 mg、scale 1.02/0.98/1.005）造六个面的读数；
- 检查算出来的 bias/scale 与真值一致（误差 < 0.1 mg / 0.001）；
- 检查超范围（某面没摆正）会被拒绝，而不是悄悄写进板子。

    python tools/host_test/test_acccal.py
"""

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import att_gui  # noqa: E402

TRUE_BIAS_MG = (20.0, -15.0, 8.0)
TRUE_SCALE = (1.02, 0.98, 1.005)


def fake_faces(bias_mg=TRUE_BIAS_MG, scale=TRUE_SCALE, bad_axis=None):
    """按 g = (gain*±1000 + bias) 造六个面的原始读数（mg）。"""
    faces = {}
    for i, (axis, up, dn) in enumerate(att_gui.Recorder.AXES):
        for name, sign in ((up, +1), (dn, -1)):
            v = [0.0, 0.0, 0.0]
            gain = (1.0 / scale[i]) if bad_axis is None or i != bad_axis else 0.6
            v[i] = gain * 1000.0 * sign + bias_mg[i]
            faces[name] = tuple(v)
    return faces


def make_recorder(faces):
    r = object.__new__(att_gui.Recorder)
    r.faces = faces
    r.last_acc_cal = None
    r.sent = []
    r.send = lambda cmd: r.sent.append(cmd)
    return r


def main():
    failures = 0

    def check(cond, what):
        nonlocal failures
        print(("    [ok]   " if cond else "    [FAIL] ") + what)
        failures += 0 if cond else 1

    print("[1] 已知真值的六面数据")
    r = make_recorder(fake_faces())
    res = r.apply_accel_cal()
    print("\n".join("    " + l for l in res["text"].splitlines()))
    check(res["ok"], "计算成功")
    check(len(r.sent) == 1, "只发出一条 ACC_CAL 命令")
    parts = r.sent[0].split(",")
    bias = [float(x) for x in parts[1:4]]
    scale = [float(x) for x in parts[4:7]]
    print(f"    bias={bias}  scale={scale}")
    check(all(abs(b * 1000 - t) < 0.1 for b, t in zip(bias, TRUE_BIAS_MG)),
          "bias 与真值一致（<0.1 mg）")
    check(all(abs(s - t) < 0.001 for s, t in zip(scale, TRUE_SCALE)),
          "scale 与真值一致（<0.001）")
    norm_err = []
    for name, raw in r.faces.items():
        corr = [(raw[i] - bias[i] * 1000.0) * scale[i] for i in range(3)]
        norm_err.append(abs(math.sqrt(sum(c * c for c in corr)) / 1000.0 - 1.0))
    check(max(norm_err) < 0.002, "校正后每个面的 |a| 都在 1 g ±0.2% 内")

    print("\n[2] 缺面")
    r = make_recorder({"+z": (0, 0, 1000)})
    res = r.apply_accel_cal()
    check(not res["ok"] and "还差" in res["text"], "缺面时拒绝并说明缺哪个")
    check(not r.sent, "没有发出命令")

    print("\n[3] 某个面没摆正（Z 轴 span 只有 1200 mg）")
    r = make_recorder(fake_faces(bad_axis=2))
    res = r.apply_accel_cal()
    check(not res["ok"] and "没摆正" in res["text"], "数据不可信时拒绝")
    check(not r.sent, "没有发出命令")

    print("\n" + ("全部通过" if failures == 0 else f"有 {failures} 项失败"))
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
