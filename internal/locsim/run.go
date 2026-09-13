package locsim

import (
	"sort"

	"github.com/Rione/ssl-RACOON-Pi2/internal/localization"
)

// RunFilter は合成したセンサ列を推定器へ流し、各車輪周期の出力を返す。
//
// 実機と同じ順序で入れるのが肝である:
//
//   - 車輪は 8 ms ごとに、サンプル時刻付きで届く。
//   - vision は**到着時刻**に届く。中身の時刻 (露光時刻) はそれより過去。
//
// vision を露光時刻の順に入れてしまうと、遅延観測の扱い (OOSM) を
// 一切試さないまま「動いた」ことになる。
func RunFilter(s *Sensors, cfg localization.Config, opts localization.EstimatorOptions) ([]localization.Estimate, *localization.Estimator, error) {
	est, err := localization.NewEstimator(cfg, opts)
	if err != nil {
		return nil, nil, err
	}

	vision := append([]localization.VisionPose(nil), s.Vision...)
	sort.Slice(vision, func(i, j int) bool { return vision[i].Arrival < vision[j].Arrival })

	// 車輪サンプルの時刻は「STM が読んだ推定時刻」で、Pi に届くのは
	// timeOffset ぶん後の転送時刻。到着順を決めるのはそちら。
	offset := localization.Stamp(-s.Config.WheelTimeOffset)

	out := make([]localization.Estimate, 0, len(s.Wheels))
	vi := 0
	for _, w := range s.Wheels {
		arrivedBy := w.Stamp + offset
		for vi < len(vision) && vision[vi].Arrival <= arrivedBy {
			est.AddVision(vision[vi])
			vi++
		}
		est.AddWheel(w)
		out = append(out, est.Current())
	}
	for vi < len(vision) {
		est.AddVision(vision[vi])
		vi++
	}
	return out, est, nil
}

// Evaluate は 1 シナリオを走らせて評価結果をまとめる。
func Evaluate(name string, s *Sensors, cfg localization.Config, opts localization.EstimatorOptions) (Report, error) {
	est, _, err := RunFilter(s, cfg, opts)
	if err != nil {
		return Report{}, err
	}
	return BuildReport(name, s, est), nil
}

// BuildReport は推定列から評価結果を組み立てる。
//
// 立ち上がりは除く。最初の vision が来るまで位置は未知なので、
// そこを含めると「フィルタの性能」ではなく「初期化の速さ」を測ってしまう。
func BuildReport(name string, s *Sensors, est []localization.Estimate) Report {
	warmup := len(est) / 10
	if warmup < 1 {
		warmup = 1
	}
	if warmup >= len(est) {
		warmup = 0
	}
	trimmed := est[warmup:]

	times := make([]localization.Stamp, 0, len(trimmed))
	for _, e := range trimmed {
		times = append(times, e.Stamp)
	}

	fused := EvaluateEstimates(s, trimmed)
	raw := EvaluateVisionRaw(s, times)
	return Report{
		Scenario:    name,
		Raw:         raw,
		Fused:       fused,
		Improvement: Improvement(raw, fused),
		Consistency: EvaluateConsistency(s, trimmed),
	}
}
