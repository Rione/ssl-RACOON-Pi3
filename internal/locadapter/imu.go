package locadapter

import "github.com/Rione/ssl-RACOON-Pi3/internal/localization"

// IMUはプロファイルが定義する観測だけ有効にする。未定のwire仕様を推測しない。
// この変換とEstimator.AddImuの融合実装は別の責務（融合は現在未実装）。
func (r *SPIRecorder) fillImuSample(out *Sample, transfer localization.Stamp) {
	out.IMU.Stamp = transfer.Add(r.bind.ImuTimeOffset)
	if r.bind.HasGyro() {
		out.IMU.GyroZ = r.values.At(r.bind.GyroZ)
		out.IMU.HasGyro = true
	}
	if r.bind.HasAccel() {
		out.IMU.Accel.X = r.values.At(r.bind.AccelX)
		out.IMU.Accel.Y = r.values.At(r.bind.AccelY)
		out.IMU.HasAccel = true
	}
}
