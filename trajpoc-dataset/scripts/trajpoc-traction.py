#!/usr/bin/env python3
"""記録から traction (車輪が言う移動量に対して、実際に進んだ割合) を出す。

床の状態は場所で変わる。どこでも走れるようにするには、まず「どれだけ変わるか」を
知る必要がある。この道具は PoC の CSV から 0.5 s 窓ごとの比を出し、
その走行がどれだけ滑っていたかを 1 行で言う。

    trajpoc-traction.py trajpoc-*.csv

読み方 (実測、2026-09-24):
    模様の取り違え   中央 0.00    ← vision が別の物を見ている。比はずっと 0
    激しい滑り       中央 0.81    最小 0.12
    正常             中央 0.95    最小 0.81

中央値が 0.9 を切ったら、その場所は滑っている。0 のまま続くなら vision の取り違え。
"""
import csv, math, sys, statistics

SIN55 = math.sin(math.radians(55.4))
COS55 = math.cos(math.radians(55.4))
WHEEL_R = 0.0285
WINDOW = 0.5


def body_speed(w):
    """4 輪の角速度から機体の並進速度 [m/s] (回転成分は打ち消える組み合わせ)。"""
    fwd = (-w[0] - w[1] + w[2] + w[3]) / 4 * WHEEL_R / SIN55
    lat = (w[0] - w[1] - w[2] + w[3]) / 4 * WHEEL_R / COS55
    return math.hypot(fwd, lat)


def ratios(path):
    T, P, W, B = [], [], [], []
    for r in csv.DictReader(open(path)):
        try:
            T.append(float(r["t_s"]))
            P.append((float(r["pose_x_mm"]) / 1000, float(r["pose_y_mm"]) / 1000))
            W.append([float(r[k]) for k in
                      ("wheel_fl_rad_s", "wheel_bl_rad_s", "wheel_br_rad_s", "wheel_fr_rad_s")])
            B.append(float(r.get("battery_v") or 0))
        except (ValueError, KeyError):
            continue
    out = []
    for i in range(len(T)):
        j = i
        while j + 1 < len(T) and T[j + 1] - T[i] < WINDOW:
            j += 1
        if T[j] - T[i] < WINDOW * 0.8:
            continue
        dist_w = sum(body_speed(W[k]) * (T[k + 1] - T[k]) for k in range(i, j))
        if dist_w <= 0.08:          # 動いていない窓は比を取れない
            continue
        dv = math.hypot(P[j][0] - P[i][0], P[j][1] - P[i][1])
        out.append(dv / dist_w)
    volts = [v for v in B if v > 0]
    return out, (statistics.mean(volts) if volts else None)


def main(paths):
    print(f"{'記録':<46}{'窓':>5}{'中央':>7}{'最小':>7}{'10%':>7}{'電池':>7}  判定")
    for p in paths:
        rs, v = ratios(p)
        if not rs:
            print(f"{p.split('/')[-1]:<46}    -  (動いている窓が無い)")
            continue
        rs.sort()
        med, lo = statistics.median(rs), rs[0]
        p10 = rs[max(0, len(rs) // 10)]
        if med < 0.2:
            verdict = "★ vision が別の物を見ている疑い"
        elif med < 0.9:
            verdict = "滑っている"
        else:
            verdict = "正常"
        vs = f"{v:5.1f}V" if v else "  不明"
        print(f"{p.split('/')[-1]:<46}{len(rs):5d}{med:7.2f}{lo:7.2f}{p10:7.2f}{vs:>7}  {verdict}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    main(sys.argv[1:])
