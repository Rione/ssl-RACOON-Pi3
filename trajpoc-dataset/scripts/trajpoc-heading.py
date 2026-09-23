#!/usr/bin/env python3
"""trajpoc の CSV から、向きのぶれ (向きを変えない軌道なのに機体が回る) を調べる。

1. いつずれるか: 向きの誤差の最大と、その時刻が走り出し / 巡航 / 止まり際のどこか
2. 何に引きずられて回るか: vision で測った角速度から「指令した角速度 (遅れ分ずらす)」を引いた
   外乱の角速度を、機体座標での指令の加速度・速度に最小二乗で当てはめる
3. 回っていることで横にずれるか: 指令が効く頃には機体が ω·遅れ だけ回っているので、
   並進の向きがずれる。その予測 (v × ω × 遅れ) と、実際の横方向の速度の誤差を比べる

  scripts/trajpoc-heading.py [--delay 0.09] trajpoc-results/trajpoc-*.csv
"""
import argparse
import csv
import math
import os

import numpy as np


def load(path):
    rows = [r for r in csv.DictReader(open(path)) if r["held"] == "0"]
    f = lambda r, k: float(r[k])
    cmd = {k: np.array([f(r, k) for r in rows]) for k in
           ("t_s", "cmd_world_vx_mm_s", "cmd_world_vy_mm_s", "cmd_omega_rad_s", "cmd_body_vx_mm_s", "cmd_body_vy_mm_s")}
    seen, vis = set(), []
    for r in rows:  # vision の撮影ごとに 1 点
        tv = f(r, "tv_s")
        if tv in seen:
            continue
        seen.add(tv)
        vis.append([tv, f(r, "pose_x_mm"), f(r, "pose_y_mm"), f(r, "pose_theta_rad"), f(r, "ref_theta_rad"),
                    f(r, "ref_vx_mm_s"), f(r, "ref_vy_mm_s")])
    vis = np.array(sorted(vis))
    return cmd, vis


def deriv(t, y, half=0.03):
    """±half 秒の中心差分。"""
    out = np.full_like(y, np.nan)
    for i in range(len(t)):
        a = np.searchsorted(t, t[i] - half)
        b = np.searchsorted(t, t[i] + half) - 1
        if a < i < b:
            out[i] = (y[b] - y[a]) / (t[b] - t[a])
    return out


def interp(t, tt, yy):
    return np.interp(t, tt, yy)


def analyze(path, delay):
    cmd, vis = load(path)
    tv, px, py, th, rth, rvx, rvy = vis.T
    th_u = np.unwrap(th)
    err = np.degrees(np.array([math.remainder(a - b, 2 * math.pi) for a, b in zip(th, rth)]))
    moving = np.hypot(rvx, rvy) > 50
    t_end = tv[moving].max() if moving.any() else tv.max()
    t_start = tv[moving].min() if moving.any() else 0

    # 1. いつずれるか
    i_max = int(np.nanargmax(np.abs(err)))
    tm = tv[i_max]
    ref_speed = np.hypot(rvx, rvy)
    if tm < t_start or tm > t_end:
        phase = "静止中 (走る前 / 後)"
    else:
        acc = deriv(tv, ref_speed)[i_max]
        phase = "加速中" if acc > 200 else ("減速中" if acc < -200 else "巡航中")

    # 2. 外乱の角速度 = 測った角速度 − 遅れ分ずらした指令の角速度
    w_meas = deriv(tv, th_u)
    t_cmd = cmd["t_s"]
    w_cmd = interp(tv - delay, t_cmd, cmd["cmd_omega_rad_s"])
    dist = w_meas - w_cmd
    # 機体座標の指令 (遅れ分ずらす) と、その時間微分 (加速度)
    bx = interp(tv - delay, t_cmd, cmd["cmd_body_vx_mm_s"])
    by = interp(tv - delay, t_cmd, cmd["cmd_body_vy_mm_s"])
    abx, aby = deriv(tv, bx, 0.04), deriv(tv, by, 0.04)
    ok = ~np.isnan(dist) & ~np.isnan(abx) & ~np.isnan(aby) & (tv > t_start) & (tv < t_end + 0.3)
    X = np.column_stack([abx[ok] / 1000, aby[ok] / 1000, bx[ok] / 1000, by[ok] / 1000, np.ones(ok.sum())])
    y = dist[ok]
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    pred = X @ coef
    r2 = 1 - np.sum((y - pred) ** 2) / np.sum((y - y.mean()) ** 2)
    # 加速度だけ / 速度だけで当てはめたときの説明力も出す (どちらが効いているか)
    def r2_of(cols):
        Xs = X[:, cols + [4]]
        c, *_ = np.linalg.lstsq(Xs, y, rcond=None)
        return 1 - np.sum((y - Xs @ c) ** 2) / np.sum((y - y.mean()) ** 2)

    # 3. 回っていることによる横ずれ: 予測 = 並進速度 × 角速度 × 遅れ (向きのずれ角 ω·遅れ)
    vx_w, vy_w = deriv(tv, px), deriv(tv, py)
    cvx = interp(tv - delay, t_cmd, cmd["cmd_world_vx_mm_s"])
    cvy = interp(tv - delay, t_cmd, cmd["cmd_world_vy_mm_s"])
    sp = np.hypot(cvx, cvy)
    with np.errstate(invalid="ignore", divide="ignore"):
        # 指令の向きに対して左向きを正とした、実際の速度の横成分
        lat = (-(vx_w) * cvy + (vy_w) * cvx) / sp
    lat_pred = sp * w_meas * delay  # 回っている分だけ、効いた時の向きが ω·遅れ ずれる
    m3 = ok & (sp > 100) & ~np.isnan(lat)
    if m3.sum() > 10:
        c3 = np.corrcoef(lat[m3], lat_pred[m3])[0, 1]
        slope = np.polyfit(lat_pred[m3], lat[m3], 1)[0]
    else:
        c3 = slope = float("nan")
    return {
        "name": os.path.basename(path), "rms": math.sqrt(np.mean(err[tv > 0] ** 2)), "max": err[i_max],
        "t_max": tm, "phase": phase, "coef": coef, "r2": r2,
        "r2_acc": r2_of([0, 1]), "r2_vel": r2_of([2, 3]),
        "dist_rms": math.sqrt(np.mean(y ** 2)), "lat_corr": c3, "lat_slope": slope,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--delay", type=float, default=0.09, help="指令が効くまでの遅れ [s] (PoC の実測で 0.085〜0.1)")
    ap.add_argument("csv", nargs="+")
    a = ap.parse_args()
    for p in a.csv:
        r = analyze(p, a.delay)
        k = r["coef"]
        print(f"== {r['name']}")
        print(f"  向きの誤差: RMS {r['rms']:.2f}°, 最大 {r['max']:+.2f}° (t={r['t_max']:.2f}s, {r['phase']})")
        print(f"  外乱の角速度: RMS {math.degrees(r['dist_rms']):.1f}°/s。当てはめ R²={r['r2']:.2f} "
              f"(加速度だけ {r['r2_acc']:.2f} / 速度だけ {r['r2_vel']:.2f})")
        print(f"    係数: 前後の加速度 {k[0]:+.3f}, 左右の加速度 {k[1]:+.3f} [rad/s per m/s²], "
              f"前後の速度 {k[2]:+.3f}, 左右の速度 {k[3]:+.3f} [rad/s per m/s], 定数 {k[4]:+.3f} rad/s")
        print(f"  回っていることによる横ずれ: 予測と実測の相関 {r['lat_corr']:.2f}, 傾き {r['lat_slope']:.2f} (1 なら説明どおり)")


if __name__ == "__main__":
    main()
