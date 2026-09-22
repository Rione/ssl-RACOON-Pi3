#!/usr/bin/env python3
"""Plot Wheel(raw) from racoon-pi2 -ds log lines or WHEEL_RAW CSV.

Usage:
  ./racoon-pi2 -ds 2>&1 | python3 scripts/wheel-raw-graph.py
  python3 scripts/wheel-raw-graph.py /path/to/log.txt

Parses lines like:
  [SPI RX] Wheel(raw) FL: 13, BL: -1271, BR: 22, FR: 190
  [Serial RX] Wheel(raw) FL: 13, BL: -1271, BR: 22, FR: 190
"""

import re
import sys
from collections import deque

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError as e:
    print("matplotlib is required: pip install matplotlib", file=sys.stderr)
    raise SystemExit(1) from e

WHEEL_LINE = re.compile(
    r"Wheel\(raw\) FL: (-?\d+), BL: (-?\d+), BR: (-?\d+), FR: (-?\d+)"
)
CSV_LINE = re.compile(r"^WHEEL_RAW,(-?\d+),(-?\d+),(-?\d+),(-?\d+)")

MAX_POINTS = 600


def parse_line(line):
    m = WHEEL_LINE.search(line)
    if m:
        return tuple(int(v) for v in m.groups())
    m = CSV_LINE.match(line.strip())
    if m:
        return tuple(int(v) for v in m.groups())
    return None


def read_all(path):
    samples = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            row = parse_line(line)
            if row:
                samples.append(row)
    return samples


def main():
    if len(sys.argv) > 1:
        samples = read_all(sys.argv[1])
        if not samples:
            print("No Wheel(raw) lines found.", file=sys.stderr)
            raise SystemExit(1)
        fl, bl, br, fr = zip(*samples)
        fig, ax = plt.subplots()
        ax.plot(fl, label="FL")
        ax.plot(bl, label="BL")
        ax.plot(br, label="BR")
        ax.plot(fr, label="FR")
        ax.legend()
        ax.set_xlabel("sample #")
        ax.set_ylabel("raw")
        ax.set_title("Wheel(raw)")
        plt.show()
        return

    fl = deque(maxlen=MAX_POINTS)
    bl = deque(maxlen=MAX_POINTS)
    br = deque(maxlen=MAX_POINTS)
    fr = deque(maxlen=MAX_POINTS)

    fig, ax = plt.subplots()
    (l_fl,) = ax.plot([], [], label="FL")
    (l_bl,) = ax.plot([], [], label="BL")
    (l_br,) = ax.plot([], [], label="BR")
    (l_fr,) = ax.plot([], [], label="FR")
    ax.legend(loc="upper left")
    ax.set_xlabel("sample #")
    ax.set_ylabel("raw")
    ax.set_title("Wheel(raw) live")

    def update(_frame):
        while True:
            line = sys.stdin.readline()
            if not line:
                return l_fl, l_bl, l_br, l_fr
            row = parse_line(line)
            if row:
                fl.append(row[0])
                bl.append(row[1])
                br.append(row[2])
                fr.append(row[3])
                break
        xs = list(range(len(fl)))
        l_fl.set_data(xs, list(fl))
        l_bl.set_data(xs, list(bl))
        l_br.set_data(xs, list(br))
        l_fr.set_data(xs, list(fr))
        ax.relim()
        ax.autoscale_view()
        return l_fl, l_bl, l_br, l_fr

    print("Reading Wheel(raw) from stdin… (pipe ./racoon-pi2 -ds output here)", file=sys.stderr)
    FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
