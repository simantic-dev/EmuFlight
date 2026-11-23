"""Render arm.gif from a `sim run --record` bundle of the EmuFlight MATEKF405SE run.
Rows: S.Bus AUX1 (uart.jsonl frames, 14 ms) and the four motor pins (gpio.jsonl, DShot600
decoded like dshot_decode.py). Every number on screen is read from the bundle.
Usage: uv run --with pillow make_gif.py <rec_dir> [out.gif]"""
import json, sys
from PIL import Image, ImageDraw

rec, out = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else "arm.gif")
MOTORS = [(7, "M1 PB7"), (6, "M2 PB6"), (0, "M3 PB0"), (1, "M4 PB1")]

edges = {p: [] for p, _ in MOTORS}
for l in open(f"{rec}/gpio.jsonl"):
    if '"gpioPortB"' not in l: continue
    d = json.loads(l)
    if d["pin"] in edges: edges[d["pin"]].append((d["t"], d["level"]))

def frames(e):                                    # [(t_first_rise, value, crc_ok)]
    res, cur, rise, last = [], [], None, None
    def flush():
        if len(cur) != 16: return
        v = 0
        for _, w in cur: v = (v << 1) | (w > 940)
        v12 = ((v >> 5) << 1) | ((v >> 4) & 1)
        res.append((cur[0][0], v >> 5, ((v12 ^ (v12 >> 4) ^ (v12 >> 8)) & 15) == v & 15))
    for t, lv in e:
        if lv: rise = t
        elif rise is not None:
            if cur and rise - last > 5e-6: flush(); cur = []
            cur.append((rise, (t - rise) * 1e9)); last, rise = rise, None
    flush()
    return res
fr = {p: frames(e) for p, e in edges.items()}

# the first S.Bus frame whose bytes differ from the one before it is AUX1 low -> high (peer sends 172, then 1811)
prev, t_aux = None, None
for l in open(f"{rec}/uart.jsonl"):
    d = json.loads(l)
    if prev is not None and d["text"] != prev and t_aux is None: t_aux = d["t"]
    prev = d["text"]
sbus_t = [json.loads(l)["t"] for l in open(f"{rec}/uart.jsonl")]
t_arm = min(t for t, v, ok in fr[7] if v == 138)
t_dis = min(t for t, v, ok in fr[7] if v == 0)
tot = {p: len(f) for p, f in fr.items()}
good = {p: sum(ok for _, _, ok in f) for p, f in fr.items()}

T0, T1 = t_aux - 0.030, t_arm + 0.030
W, H, L = 960, 470, 130
x = lambda t: L + (t - T0) / (T1 - T0) * (W - L - 20)
rows = {"AUX1 (S.Bus ch5)": 90, **{n: 170 + 55 * i for i, (_, n) in enumerate(MOTORS)}}
N = 60
imgs = []
for i in range(N):
    tn = T0 + (T1 - T0) * (i + 1) / N
    im = Image.new("RGB", (W, H), "white"); d = ImageDraw.Draw(im)
    d.text((10, 10), "EmuFlight MATEKF405SE on simulated STM32F405RG (pyrite sim 0.10.6): S.Bus AUX1 -> arm -> DShot600", fill="black")
    state = "ARMED" if tn >= t_arm else "disarmed"
    d.text((10, 30), f"virtual t = {tn:7.4f} s   {state}", fill="#c33" if state == "ARMED" else "black")
    y = rows["AUX1 (S.Bus ch5)"]; d.text((10, y - 6), "AUX1 (S.Bus ch5)", fill="black")
    for t in sbus_t:
        if T0 <= t <= tn: d.line([(x(t), y + 6), (x(t), y + 14)], fill="#999")
    lvl = [(T0, 172)] + ([(t_aux, 1811)] if tn >= t_aux else [])
    px, py = x(T0), y + 25
    for t, v in lvl[1:]:
        d.line([(px, py), (x(t), py), (x(t), y - 10)], fill="#36c", width=2); px, py = x(t), y - 10
    d.line([(px, py), (x(tn), py)], fill="#36c", width=2)
    d.text((L + 5, y + 26), "172 (low)", fill="#36c")
    if tn >= t_aux: d.text((x(t_aux) + 6, y - 24), f"1811 (high) at {t_aux:.4f} s", fill="#36c")
    for p, name in MOTORS:
        y = rows[name]; d.text((10, y - 6), name, fill="black")
        n = 0
        for t, v, ok in fr[p]:
            if T0 <= t <= tn:
                n += 1
                d.rectangle([x(t), y - (14 if v else 3), x(t) + 1, y + 6], fill="#2b7" if v else "#aaa")
        cur = [v for t, v, ok in fr[p] if t <= tn]
        d.text((L + 5, y + 10), f"{n} frames in window, last value {cur[-1] if cur else '-'}", fill="black")
    d.text((10, H - 70), f"first DShot value 0 at {t_dis:.4f} s (disarmed); first value 138 at {t_arm:.4f} s, "
           f"{(t_arm - t_aux) * 1e3:.1f} ms after AUX1 goes high", fill="black")
    d.text((10, H - 50), "whole run (14 s): " + ", ".join(f"{n}: {tot[p]} frames, {good[p]} CRC ok" for p, n in MOTORS), fill="black")
    d.text((10, H - 30), "DShot600 CRC checked per frame; value 138 = idle throttle (EmuFlight digital idle)", fill="#555")
    imgs.append(im)
imgs[0].save(out, save_all=True, append_images=imgs[1:] + [imgs[-1]] * 25, duration=100, loop=0, optimize=True)
print(out, N, t_aux, t_arm, t_dis, tot, good)
