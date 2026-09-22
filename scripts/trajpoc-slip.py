#!/usr/bin/env python3
"""trajpoc の CSV から、タイヤのスリップと STM の車輪制御の行き過ぎを見分ける。

vision で測った機体の速度 (機体座標の vx, vy, ω) から、滑っていなければ各車輪がどれだけ回るはずかを
最小二乗で当てはめる (車輪 = J · 機体速度、J は 4×3)。車輪の角度・半径・符号は未同定なので、
記録そのものから J を決める。走りの大半は滑っていないので、J は滑っていない関係を表す。

- 車輪の実測と「機体の動きから予想した車輪の回転」のずれ = 滑り (車輪が機体と違う動きをしている)
- 車輪も機体と同じように動いている (ずれが小さい) のに機体が指令と違う = STM の車輪制御の問題

  scripts/trajpoc-slip.py [--from 2.6 --to 3.9] trajpoc-results/trajpoc-*.csv
"""
import argparse
import csv
import math
import os

import numpy as np

WHEELS = ("fl", "bl", "br", "fr")


def load(path):
    rows = [r for r in csv.DictReader(open(path)) if r["held"] == "0"]
    if "wheel_fl_rad_s" not in rows[0]:
        raise SystemExit(f"{path}: 車輪の列が無い (車輪を記録する前の版の記録)")
    f = float
    t = np.array([f(r["t_s"]) for r in rows])
    wh = np.array([[f(r[f"wheel_{w}_rad_s"]) for w in WHEELS] for r in rows])
    cmd = np.array([[f(r["cmd_body_vx_mm_s"]), f(r["cmd_body_vy_mm_s"]), f(r["cmd_omega_rad_s"])] for r in rows])
    seen, vis = set(), []
    for r in rows:  # vision の撮影ごとに 1 点
        tv = f(r["tv_s"])
        if tv in seen:
            continue
        seen.add(tv)
        vis.append([tv, f(r["pose_x_mm"]), f(r["pose_y_mm"]), f(r["pose_theta_rad"])])
    vis = np.array(sorted(vis))
    return t, wh, cmd, vis


def body_velocity(vis, half=0.02):
    """±half 秒の中心差分で、機体座標の速度 [mm/s] と角速度 [rad/s]。"""
    tv, x, y, th = vis.T
    thu = np.unwrap(th)
    out = np.full((len(tv), 3), np.nan)
    for i in range(len(tv)):
        a = np.searchsorted(tv, tv[i] - half)
        b = np.searchsorted(tv, tv[i] + half) - 1
        if not (a < i < b):
            continue
        dt = tv[b] - tv[a]
        vx, vy = (x[b] - x[a]) / dt, (y[b] - y[a]) / dt
        c, s = math.cos(th[i]), math.sin(th[i])
        out[i] = [c * vx + s * vy, -s * vx + c * vy, (thu[b] - thu[a]) / dt]
    return out


def analyze(path, t_from, t_to, delay):
    t, wh, cmd, vis = load(path)
    tv = vis[:, 0]
    body = body_velocity(vis)
    # 車輪の値は SPI の周期ごと。vision の撮影時刻に合わせる (車輪の値は 1 周期 = 8 ms 古い)
    w_at = np.column_stack([np.interp(tv + 0.008, t, wh[:, k]) for k in range(4)])
    ok = ~np.isnan(body).any(axis=1) & (tv > 0)
    X = np.column_stack([body[ok, 0] / 1000, body[ok, 1] / 1000, body[ok, 2]])  # m/s, m/s, rad/s
    J, *_ = np.linalg.lstsq(X, w_at[ok], rcond=None)  # 3×4: 車輪 = 機体速度 · J
    pred = np.full_like(w_at, np.nan)
    pred[ok] = X @ J
    res = w_at - pred
    r2 = [1 - np.nanvar(res[ok, k]) / np.nanvar(w_at[ok, k]) for k in range(4)]

    print(f"== {os.path.basename(path)}")
    print("  機体速度 → 車輪の当てはめ [rad/s per m/s, rad/s per rad/s]:")
    for k, w in enumerate(WHEELS):
        print(f"    {w}: vx {J[0, k]:+6.1f}  vy {J[1, k]:+6.1f}  ω {J[2, k]:+6.2f}   R² {r2[k]:.3f}  "
              f"ずれ RMS {np.sqrt(np.nanmean(res[ok, k] ** 2)):.2f} rad/s")
    # 走り方ごとのずれ: 車輪の予想の速さで割った比 (大きいほど滑っている)
    sp = np.hypot(body[:, 0], body[:, 1])
    cmd_at = np.column_stack([np.interp(tv - delay, t, cmd[:, k]) for k in range(3)])
    print(f"  時系列 ({t_from}〜{t_to} s、機体の前後 = vx、車輪は実測 − 予想 [rad/s]):")
    print("     t    vx実測  vx指令(遅れ)  vy実測   " + "  ".join(f"{w}ずれ" for w in WHEELS))
    last = -1.0
    for i in range(len(tv)):
        if not (t_from <= tv[i] <= t_to) or not ok[i] or tv[i] - last < 0.03:
            continue
        last = tv[i]
        print(f"    {tv[i]:4.2f}  {body[i, 0]:+6.0f}   {cmd_at[i, 0]:+6.0f}      {body[i, 1]:+6.0f}  " +
              "  ".join(f"{res[i, k]:+6.2f}" for k in range(4)))
    return res, ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from", dest="t_from", type=float, default=0.0)
    ap.add_argument("--to", dest="t_to", type=float, default=99.0)
    ap.add_argument("--delay", type=float, default=0.09, help="指令が効くまでの遅れ [s]")
    ap.add_argument("csv", nargs="+")
    a = ap.parse_args()
    for p in a.csv:
        analyze(p, a.t_from, a.t_to, a.delay)


if __name__ == "__main__":
    main()
