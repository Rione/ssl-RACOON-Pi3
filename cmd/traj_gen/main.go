// traj_gen は時刻つき軌道追従の PoC 用に、仮想の TimedPoint の列を標準出力へ書く。
// 中身は RAVEN の TimedTrajectoryPoint と同じ (x, y [mm], theta [rad], t [ns]) で、
// t は先頭からの相対時刻。座標は開始位置が原点・ロボットの前方が +x。
//
//	traj_gen -shape square -size 0.6 -speed 0.4 | ssh root@<robot> '...racoon-pi3 -trajpoc ...'
//
// 詳しい回し方は docs/traj-poc.md。
package main

import (
	"flag"
	"fmt"
	"os"

	"github.com/Rione/ssl-RACOON-Pi3/internal/trajpoc"
)

func main() {
	c := trajpoc.DefaultGenConfig()
	flag.StringVar(&c.Shape, "shape", c.Shape, "line | square | circle | fig8 | turn (その場で回る。size=角度[rad], speed=角速度[rad/s], accel=角加速度[rad/s^2]) | hold (止まったまま size 秒)")
	flag.Float64Var(&c.Size, "size", c.Size, "形の大きさ [m] (line=片道, square=一辺, circle=直径, fig8=輪1つの直径)")
	flag.Float64Var(&c.Speed, "speed", c.Speed, "巡航速度 [m/s]")
	flag.Float64Var(&c.Accel, "accel", c.Accel, "加減速度・横加速度の上限 [m/s^2]")
	flag.Float64Var(&c.Dt, "dt", c.Dt, "点の間隔 [s]")
	flag.StringVar(&c.Heading, "heading", c.Heading, "fixed | tangent (tangent は circle/fig8 のみ)")
	flag.IntVar(&c.Laps, "laps", c.Laps, "周回数")
	flag.Parse()

	knots, err := trajpoc.Generate(c)
	if err != nil {
		fmt.Fprintln(os.Stderr, "traj_gen:", err)
		os.Exit(2)
	}
	b := trajpoc.MeasureBounds(knots)
	fmt.Fprintf(os.Stderr, "traj_gen: %s size=%.2fm speed=%.2fm/s accel=%.2fm/s^2 heading=%s laps=%d -> %d points, %.2f s, "+
		"max radius %.2f m, max speed %.2f m/s, max yaw rate %.2f rad/s\n",
		c.Shape, c.Size, c.Speed, c.Accel, c.Heading, c.Laps, len(knots), b.Duration, b.MaxRadius, b.MaxSpeed, b.MaxYawRate)
	comment := fmt.Sprintf("traj_gen shape=%s size=%g speed=%g accel=%g dt=%g heading=%s laps=%d",
		c.Shape, c.Size, c.Speed, c.Accel, c.Dt, c.Heading, c.Laps)
	if err := trajpoc.WriteJSONL(os.Stdout, knots, comment); err != nil {
		fmt.Fprintln(os.Stderr, "traj_gen:", err)
		os.Exit(1)
	}
}
