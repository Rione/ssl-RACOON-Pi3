package locadapter

import "github.com/Rione/ssl-RACOON-Pi3/internal/localization"

// 車輪の単位はプロファイル適用済みrad/s、並びはSTMスロット順。
// 論理輪への並べ替えはEstimator.AddWheel内で一度だけ行う。
func (r *SPIRecorder) fillWheelSample(out *Sample, transfer localization.Stamp) {
	out.Wheel.Stamp = transfer.Add(r.bind.WheelTimeOffset)
	for slot, idx := range r.bind.WheelSlots {
		out.Wheel.Omega[slot] = r.values.At(idx)
		out.WheelRaw[slot] = r.values.RawAt(idx)
	}
}
