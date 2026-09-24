#!/usr/bin/env python3
"""trajpoc の CSV から、円の軌道の「半径方向のずれ」と「速度の比」を出す。

円 (circle) は 1 輪、8 の字 (fig8) は 2 輪 (前半が左回り、後半が右回り) として、
軌道の時間を輪の数で等分し、輪ごとに参照点列へ円を当てはめて
「vision の位置の中心からの距離 − 半径」を集計する。正 = 外へ膨らむ、負 = 内へ切れ込む。

  scripts/trajpoc-circle.py --loops 1 trajpoc-results/trajpoc-*-ffp-hermite.csv
  scripts/trajpoc-circle.py --loops 2 trajpoc-results/trajpoc-<fig8 の時刻>-ffp-hermite.csv
"""
import argparse
import csv
import math
import os

import numpy as np

# 走り出しと止まりの加減速を除くための余白 [s]
EDGE = 0.3


def fit_circle(xs, ys):
    """代数的な最小二乗で円を当てはめる。(cx, cy, R)"""
    x, y = np.asarray(xs), np.asarray(ys)
    a = np.column_stack([x, y, np.ones_like(x)])
    b = x * x + y * y
    d, e, f = np.linalg.lstsq(a, b, rcond=None)[0]
    cx, cy = d / 2, e / 2
    return cx, cy, math.sqrt(f + cx * cx + cy * cy)


def analyze(path, loops):
    rows = [r for r in csv.DictReader(open(path)) if r["held"] == "0"]
    seen, pts = set(), []
    for r in rows:  # vision の撮影ごとに 1 点 (同じ撮影は 125 Hz の周期で重複する)
        tv = float(r["tv_s"])
        if tv in seen or tv < 0:
            continue
        seen.add(tv)
        pts.append({k: float(r[k]) for k in (
            "tv_s", "pose_x_mm", "pose_y_mm", "ref_x_mm", "ref_y_mm", "ref_vx_mm_s", "ref_vy_mm_s")})
    t_end = max(p["tv_s"] for p in pts if math.hypot(p["ref_vx_mm_s"], p["ref_vy_mm_s"]) > 1)
    out = []
    for i in range(loops):
        a, b = t_end * i / loops, t_end * (i + 1) / loops
        seg = [p for p in pts if a <= p["tv_s"] <= b]
        cx, cy, radius = fit_circle([p["ref_x_mm"] for p in seg], [p["ref_y_mm"] for p in seg])
        body = [p for p in seg if EDGE <= p["tv_s"] <= t_end - EDGE]
        rad = sorted(math.hypot(p["pose_x_mm"] - cx, p["pose_y_mm"] - cy) - radius for p in body)
        # 回る向き: 中心から見た参照の速度の外積の符号
        cross = sum((p["ref_x_mm"] - cx) * p["ref_vy_mm_s"] - (p["ref_y_mm"] - cy) * p["ref_vx_mm_s"] for p in body)
        # 実速度 (vision の 80 ms 中心差分) / 参照速度。加減速を除いた中ほどで比べる
        mid = [p for p in body if a + (b - a) * 0.25 <= p["tv_s"] <= a + (b - a) * 0.75]
        ratios = []
        for p in mid:
            before = [q for q in pts if q["tv_s"] <= p["tv_s"] - 0.04]
            after = [q for q in pts if q["tv_s"] >= p["tv_s"] + 0.04]
            if not before or not after:
                continue
            q0, q1 = before[-1], after[0]
            v = math.hypot(q1["pose_x_mm"] - q0["pose_x_mm"], q1["pose_y_mm"] - q0["pose_y_mm"]) / (q1["tv_s"] - q0["tv_s"])
            ratios.append(v / max(math.hypot(p["ref_vx_mm_s"], p["ref_vy_mm_s"]), 1))
        out.append({
            "loop": i + 1, "dir": "左回り" if cross > 0 else "右回り", "R": radius,
            "mean": sum(rad) / len(rad), "p5": rad[len(rad) // 20], "p95": rad[len(rad) * 19 // 20],
            "speed_ratio": sum(ratios) / len(ratios) if ratios else float("nan"), "n": len(rad),
        })
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--loops", type=int, default=1, help="輪の数 (circle=1, fig8=2)")
    ap.add_argument("csv", nargs="+")
    args = ap.parse_args()
    for path in args.csv:
        for r in analyze(path, args.loops):
            print(f"{os.path.basename(path)} 輪{r['loop']} {r['dir']} R={r['R']:.0f}mm: "
                  f"半径方向のずれ 平均 {r['mean']:+.1f} mm (5%点 {r['p5']:+.1f} / 95%点 {r['p95']:+.1f}, n={r['n']}), "
                  f"実速度/参照 {r['speed_ratio']:.3f}")


if __name__ == "__main__":
    main()
