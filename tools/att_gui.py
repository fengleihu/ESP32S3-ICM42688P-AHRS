#!/usr/bin/env python3
"""att_gui.py — 点按钮打点的测量台（本地网页界面，不需要掐时间）。

为什么有这个：靠"45 秒后开始倾斜"这种时间指令测不准，前几轮因此全废。这里改成
**你在板子摆好之后点一下按钮**，程序把 `MARK <标签>` 立刻发给板子，日志里就有了
确切时间点；分析时取"每个标记前 3 秒"作为该状态的测量窗口（标记即状态末尾）。

用法
----
    python tools/att_gui.py                 # 自动找 ESP32 的串口
    python tools/att_gui.py --port COM3     # 指定串口
    python tools/att_gui.py --http-port 8770 --out logs/session.log

启动后浏览器会打开 http://127.0.0.1:8765/ ，页面上的按钮：
    ① 水平   ② 倾斜 30°   ③ 回水平   ④ 转 180°   分析判据
每按一次就把当前状态记为 MARK；最后点「分析判据」直接看 PASS/FAIL。

也可以只用键盘：页面上按钮支持 Tab + 回车。
退出：终端里 Ctrl+C。
"""

from __future__ import annotations

import argparse
import contextlib
import io
import json
import math
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import att_report  # noqa: E402

PAGE = r"""<!doctype html>
<html lang="zh"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AHRS 测量台</title>
<style>
 :root { color-scheme: dark; }
 body { margin:0; padding:18px; font:15px/1.5 -apple-system,"Segoe UI",system-ui,sans-serif;
        background:#12161c; color:#e6edf3; }
 h1 { font-size:20px; margin:0 0 4px; }
 .sub { color:#8b98a5; margin-bottom:14px; }
 .live { display:grid; grid-template-columns:repeat(4,1fr); gap:10px; margin-bottom:16px; }
 .cell { background:#1b222c; border:1px solid #253040; border-radius:10px; padding:10px 12px; }
 .cell .k { color:#8b98a5; font-size:12px; }
 .cell .v { font-size:22px; font-variant-numeric:tabular-nums; }
 .row { display:flex; flex-wrap:wrap; gap:10px; margin-bottom:10px; }
 button { font:600 15px/1 inherit; padding:14px 18px; border-radius:10px; cursor:pointer;
          border:1px solid #2b3a4d; background:#1f2937; color:#e6edf3; }
 button:hover { background:#26313f; }
 button.primary { background:#1f6feb; border-color:#388bfd; }
 button.warn { background:#5c2a12; border-color:#8a4b23; }
 button:disabled { opacity:.45; cursor:default; }
 pre { background:#0d1117; border:1px solid #253040; border-radius:10px; padding:14px;
       overflow:auto; font-size:12.5px; line-height:1.45; min-height:160px; }
 .ok { color:#3fb950; } .bad { color:#f85149; }
</style></head>
<body>
<h1>AHRS 测量台</h1>
<div class="sub">板子摆好后点对应按钮 —— 时间点由按钮确定，不用掐秒表。</div>

<div class="live">
  <div class="cell"><div class="k">yaw</div><div class="v" id="yaw">--</div></div>
  <div class="cell"><div class="k">pitch</div><div class="v" id="pitch">--</div></div>
  <div class="cell"><div class="k">倾斜角</div><div class="v" id="tilt">--</div></div>
  <div class="cell"><div class="k">|B| µT</div><div class="v" id="bmag">--</div></div>
</div>

<div class="row">
  <button data-label="level">① 水平</button>
  <button data-label="tilt30">② 倾斜 30°</button>
  <button data-label="level2">③ 回水平</button>
  <button data-label="turn180">④ 转 180°</button>
</div>
<div class="row">
  <button class="primary" id="analyze">分析判据</button>
  <button class="warn" id="recal">重新校准磁力计</button>
  <button id="clear">清空记录，重新开始</button>
</div>

<pre id="out">连接中…</pre>
<div class="sub" id="status"></div>

<script>
const out = document.getElementById('out');
const statusEl = document.getElementById('status');
window.onerror = function (msg, src, line) {
  out.className = 'bad';
  out.textContent = 'JS 出错：' + msg + '\n' + (src || '') + ':' + line +
    '\n请把这个提示发给我（页面脚本需要修）。';
};
function pct(x){ return (x>=0?'+':'') + x.toFixed(2); }
async function refresh(){
  try {
    const r = await fetch('/state');
    const s = await r.json();
    document.getElementById('yaw').textContent   = s.att ? pct(s.att.yaw) : '--';
    document.getElementById('pitch').textContent = s.att ? pct(s.att.pitch) : '--';
    document.getElementById('tilt').textContent  = s.att ? s.att.tilt.toFixed(1) : '--';
    document.getElementById('bmag').textContent  = s.att ? s.att.bmag.toFixed(1) : '--';
    statusEl.textContent =
      s.port + ' · ' + s.att_count + ' 帧 · ' + s.marks.length + ' 个标记 · ' + s.log;
  } catch (e) {
    statusEl.textContent = '与测量台断开（服务是否已停止？）';
  }
}
async function post(path, body){
  const r = await fetch(path, {method:'POST', body: body || ''});
  return await r.json();
}
document.querySelectorAll('button[data-label]').forEach(b => {
  b.onclick = async () => {
    const j = await post('/mark/' + encodeURIComponent(b.dataset.label));
    out.className = '';
    out.textContent = '已标记：' + b.textContent + '（' + j.time + '）\n' +
      '累计标记：' + j.marks.map(m => m.label + '@' + m.time).join('  ');
  };
});
document.getElementById('analyze').onclick = async () => {
  out.textContent = '分析中…';
  const j = await post('/analyze');
  out.textContent = j.text;
  out.className = j.fail ? 'bad' : 'ok';
};
document.getElementById('recal').onclick = async () => {
  await post('/recal');
  out.className = '';
  out.textContent = '已发送 RECAL_MAG：板子会重启，看 OLED 出现 MAG CAL 后开始转 15 秒。\n' +
                    '校准完 / 重启后，之前的状态窗口要重新点一遍。';
};
document.getElementById('clear').onclick = async () => {
  await post('/clear');
  out.className = '';
  out.textContent = '已清空记录，可以重新开始。';
};
setInterval(refresh, 300); refresh();
</script>
</body></html>
"""


class Recorder:
    """串口读取 + 打点 + 分析。"""

    def __init__(self, port, baud, out_path, analyze_args):
        import serial  # 延迟导入，--help 时不需要 pyserial

        self.analyze_args = analyze_args
        self.port = port
        self.out_path = out_path
        os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
        self.fh = open(out_path, "w", encoding="utf-8")
        self.lock = threading.Lock()
        self.att_count = 0
        self.marks = []
        self.att = None
        self.ser = serial.Serial(port, baud, timeout=0.2)
        threading.Thread(target=self._reader, daemon=True).start()

    # ---- 串口 ----
    def _reader(self):
        while True:
            try:
                raw = self.ser.readline()
            except Exception:
                time.sleep(0.2)
                continue
            if not raw:
                continue
            line = raw.decode("utf-8", "replace").rstrip("\r\n")
            if not line:
                continue
            with self.lock:
                self.fh.write(line + "\n")
                self.fh.flush()
                if line.startswith("ATT,"):
                    self.att_count += 1
                    self.att = self._parse_att(line)
                elif line.startswith("MARK,"):
                    parts = line.split(",", 2)
                    if len(parts) >= 3:
                        self.marks.append({"time": parts[1] + "ms",
                                           "label": parts[2].strip()})

    @staticmethod
    def _parse_att(line):
        p = line.split(",")
        if len(p) < 19:
            return None
        try:
            ax, ay, az = float(p[7]), float(p[8]), float(p[9])
            mx, my, mz = float(p[13]), float(p[14]), float(p[15])
            n = math.sqrt(ax * ax + ay * ay + az * az)
            tilt = 0.0 if n < 1e-6 else math.degrees(
                math.acos(max(-1.0, min(1.0, az / n))))
            return {"roll": float(p[2]), "pitch": float(p[3]), "yaw": float(p[4]),
                    "tilt": tilt, "bmag": math.sqrt(mx * mx + my * my + mz * mz)}
        except (ValueError, IndexError):
            return None

    def send(self, text):
        try:
            self.ser.write((text + "\n").encode())
            return True
        except Exception:
            return False

    def mark(self, label):
        """把当前状态记为一个 MARK（板子会回一条 MARK,<ms>,<label>）。"""
        ok = self.send("MARK " + label)
        time.sleep(0.1)  # 让板子的回显先进日志
        with self.lock:
            marks = list(self.marks)
        return {"label": label, "ok": ok, "time": time.strftime("%H:%M:%S"),
                "marks": [{"label": m["label"], "time": m["time"]} for m in marks]}

    def state(self):
        with self.lock:
            return {"port": self.port, "att_count": self.att_count,
                    "att": self.att, "marks": list(self.marks),
                    "log": self.out_path}

    def clear(self):
        with self.lock:
            self.att_count = 0
            self.marks.clear()
            self.att = None
            self.fh.seek(0)
            self.fh.truncate()

    def analyze(self):
        buf = io.StringIO()
        res = None
        with contextlib.redirect_stdout(buf):
            try:
                res = att_report.analyze_file(self.out_path, self.analyze_args)
            except Exception as exc:  # 不让界面因为一次分析崩掉
                print(f"分析失败：{exc!r}")
        text = buf.getvalue()
        return {"text": text,
                "pass": (res or {}).get("pass", 0),
                "fail": (res or {}).get("fail", 0)}


def make_handler(rec):
    class Handler(BaseHTTPRequestHandler):
        def _reply(self, code, body, ctype):
            data = body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = urlparse(self.path).path
            if path in ("/", "/index.html"):
                self._reply(200, PAGE, "text/html; charset=utf-8")
            elif path == "/state":
                self._reply(200, json.dumps(rec.state()), "application/json")
            else:
                self._reply(404, "{}", "application/json")

        def do_POST(self):
            path = urlparse(self.path).path
            if path.startswith("/mark/"):
                label = unquote(path[len("/mark/"):]) or "mark"
                self._reply(200, json.dumps(rec.mark(label)), "application/json")
            elif path == "/recal":
                rec.send("RECAL_MAG")
                self._reply(200, json.dumps({"ok": True}), "application/json")
            elif path == "/clear":
                rec.clear()
                self._reply(200, json.dumps({"ok": True}), "application/json")
            elif path == "/analyze":
                self._reply(200, json.dumps(rec.analyze()), "application/json")
            else:
                self._reply(404, "{}", "application/json")

        def log_message(self, *args):
            pass

    return Handler


def pick_port():
    try:
        from serial.tools import list_ports
    except ImportError:
        return None
    cands = list(list_ports.comports())
    for p in cands:
        text = f"{p.hwid} {p.description} {p.manufacturer}".upper()
        if "303A" in text or "JTAG" in text or "ESP32" in text:
            return p.device
    return cands[0].device if cands else None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", help="串口，例如 COM3；不给就自动找 ESP32")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--http-port", type=int, default=8765)
    ap.add_argument("--out", default="logs/session.log")
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    port = args.port or pick_port()
    if not port:
        print("找不到串口，用 --port COMx 指定", file=sys.stderr)
        return 2

    rec = Recorder(port, args.baud, args.out, att_report.default_analyze_args())
    srv = ThreadingHTTPServer(("127.0.0.1", args.http_port), make_handler(rec))
    url = f"http://127.0.0.1:{args.http_port}/"
    print(f"串口 {port} 已打开；测量台：{url}")
    print(f"日志：{os.path.abspath(args.out)}")
    print("流程：① 水平 → ② 倾斜 30° → ③ 回水平 → ④ 转 180° → 分析判据")
    print("按 Ctrl+C 结束。")
    if not args.no_browser:
        import webbrowser
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n结束")
    finally:
        srv.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
