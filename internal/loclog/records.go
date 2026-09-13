package loclog

// ログのフィールド名には必ず単位を埋める (計画 §6.1)。
//
// このリポジトリでは既に「変数名は FlWheelSpeedRadS なのに中身は m/s」という
// 事故が起きていて、実際に誤差の切り分けを難しくしている (§3.9)。
// 名前に単位を書くだけで防げるので、例外なく全面採用する。

// SPIRecord は SPI 1 トランザクションの記録。
type SPIRecord struct {
	// TxStartNs / TxEndNs は conn.Tx() の直前・直後に取った単調時刻。
	TxStartNs int64 `json:"txStart_ns"`
	TxEndNs   int64 `json:"txEnd_ns"`
	// TransferNs は上記の中点。これをこのトランザクションの時刻として扱う。
	TransferNs int64 `json:"transfer_ns"`
	// DtNs は前回の転送時刻からの実測間隔。公称 8 ms との差がジッタ (計画 §5.2 / D-4)。
	DtNs int64 `json:"dt_ns"`

	TxBase64 string `json:"tx_base64"`
	RxBase64 string `json:"rx_base64"`

	// Profile はこのフレームの解釈に使ったプロファイル名。
	// 後からバイト配置が変わっても、過去ログをどう読めばよいか分かるようにする。
	Profile string `json:"profile"`

	// FrameOffsetBytes は受信窓の中で採用したフレームの先頭位置。
	FrameOffsetBytes int `json:"frameOffset_bytes"`
	// FrameCount は窓の中で有効と判定できたフレーム数。
	//
	// 2 以上のとき、採用したフレームがどの転送に対する応答か一意に決まらない。
	// 時刻が 1 周期 (8 ms) ずれ得るので、tau_stm を測るときは必ず見ること。
	FrameCount int `json:"frameCount"`
	// FrameValid はフレームが取れたか。
	FrameValid bool `json:"frameValid"`
	// FrameError は取れなかった理由。
	FrameError string `json:"frameError,omitempty"`
}

// WheelRecord はデコード後の 4 輪角速度。
type WheelRecord struct {
	// SampleNs は timeOffsetMs を適用した後の「STM がセンサを読んだ推定時刻」。
	SampleNs int64 `json:"sample_ns"`
	// TransferNs は元になった SPI 転送時刻。差が timeOffsetMs。
	TransferNs int64 `json:"transfer_ns"`

	// SPI フレーム上の並び順 (FL, BL, BR, FR) の角速度 [rad/s]。
	// この並びが実機の物理配置と合っている保証はまだない (計画 §12-A4)。
	WheelFLRadS float64 `json:"wheelFL_rad_s"`
	WheelBLRadS float64 `json:"wheelBL_rad_s"`
	WheelBRRadS float64 `json:"wheelBR_rad_s"`
	WheelFRRadS float64 `json:"wheelFR_rad_s"`

	// スケール適用前の生値。符号・スケールの同定はここから行う。
	WheelFLRaw int64 `json:"wheelFL_raw"`
	WheelBLRaw int64 `json:"wheelBL_raw"`
	WheelBRRaw int64 `json:"wheelBR_raw"`
	WheelFRRaw int64 `json:"wheelFR_raw"`

	BatteryV float64 `json:"battery_V"`
	CapPower int64   `json:"capPower_count"`
	// SensorInfo はフォトセンサ・ドリブラのビットフィールド。
	SensorInfo int64 `json:"sensorInfo_bits"`
}

// ImuRecord はジャイロ・加速度の 1 サンプル。
//
// 計画 §3.5 の通り STM 側に IMU の実装が無いため、現状このチャンネルは空のままになる。
type ImuRecord struct {
	SampleNs   int64 `json:"sample_ns"`
	TransferNs int64 `json:"transfer_ns"`

	GyroZRadS float64 `json:"gyroZ_rad_s"`
	AccelXMS2 float64 `json:"accelX_m_s2"`
	AccelYMS2 float64 `json:"accelY_m_s2"`
	HasGyro   bool    `json:"hasGyro"`
	HasAccel  bool    `json:"hasAccel"`
}

// VisionMetaRecord は vision パケットの時刻まわり。
//
// SkewPpm / OffsetNs / MappedNs は timesync (P2) が埋める。P1 の時点では
// 生の t_capture / t_sent / recv_ns だけが入り、写像はログから後で計算できる。
type VisionMetaRecord struct {
	CameraID    uint32 `json:"camera_id"`
	FrameNumber uint32 `json:"frame_number"`

	// TCaptureS / TSentS は vision PC のクロックによる秒 (パケットの生値)。
	// 差 t_sent - t_capture は同一クロック内の差なので、同期なしで正確に測れる
	// vision の処理遅延である (計画 §5.1 / D-2)。
	TCaptureS float64 `json:"t_capture_s"`
	TSentS    float64 `json:"t_sent_s"`
	// ProcessingS は TSentS - TCaptureS。
	ProcessingS float64 `json:"processing_s"`

	// RecvNs は Rock5A が UDP を受信した単調時刻。
	RecvNs int64 `json:"recv_ns"`
	// MappedNs は Rock5A 時間軸へ写した露光時刻。未写像なら 0。
	MappedNs int64 `json:"mapped_ns"`
	// SkewPpm / OffsetNs は timesync の推定値。未実装の間は 0。
	SkewPpm  float64 `json:"skew_ppm"`
	OffsetNs int64   `json:"offset_ns"`
	Mapped   bool    `json:"mapped"`

	// FrameGap は直前に受けた同一カメラのフレーム番号との差。
	// 1 が正常。2 以上なら欠番があった (計画 §5.5 / D-1)。
	// 非単調 (0 以下) なら順序逆転か重複。
	FrameGap int64 `json:"frameGap"`
	// LostFrames はこのカメラで累積した欠番数。
	LostFrames int64 `json:"lostFrames"`
	// TotalFrames はこのカメラで受けたパケット数。
	TotalFrames int64 `json:"totalFrames"`

	// SelfSeen は自機がこのフレームに写っていたか。
	SelfSeen bool `json:"selfSeen"`
	// 自機の観測値 (SelfSeen のときのみ有効)。RAVEN 内部と揃えて mm で持つ。
	SelfXMm        float64 `json:"selfX_mm"`
	SelfYMm        float64 `json:"selfY_mm"`
	SelfThetaRad   float64 `json:"selfTheta_rad"`
	SelfConfidence float64 `json:"selfConfidence"`

	// PacketBytes はパケット長。帯域と欠落の相関を見るため。
	PacketBytes int `json:"packet_bytes"`
}

// TimingRecord はループの実測時間。
type TimingRecord struct {
	// DtNs は前周期からの実測間隔。公称 8 ms からのずれがジッタ。
	DtNs int64 `json:"dt_ns"`
	// LoopNs は 1 周期の処理にかかった時間。
	LoopNs int64 `json:"loop_ns"`
	// SPINs は conn.Tx() 自体にかかった時間。
	SPINs int64 `json:"spi_ns"`
}

// StatusRecord は記録自体の健全性 (計画 §6.1)。
type StatusRecord struct {
	UptimeNs int64 `json:"uptime_ns"`
	// Written はチャンネルへ書けたメッセージ数。
	Written int64 `json:"written_count"`
	// Dropped はキューが詰まって捨てたメッセージ数。
	//
	// 記録が詰まっても推定と制御は絶対に止めない。落とした件数はここに出す。
	Dropped int64 `json:"dropped_count"`
	// WriteErrors は MCAP への書き込みが失敗した回数。
	WriteErrors int64 `json:"writeError_count"`
	// QueueDepth は現在キューに溜まっている件数。
	QueueDepth int `json:"queueDepth_count"`
	// QueueCapacity はキューの容量。
	QueueCapacity int `json:"queueCapacity_count"`
	// BytesWritten は書き出したバイト数。
	BytesWritten int64 `json:"bytesWritten_bytes"`
}

// CommandRecord は STM へ送った指令。
//
// 単位は SPI フレーム上のもの (mm/s, mrad/s) をそのまま残す。
// ここで SI へ直すと、同定のときに「どちらの単位で見ているか」が曖昧になる。
type CommandRecord struct {
	TransferNs int64 `json:"transfer_ns"`

	VelXMmS     int16 `json:"velX_mm_s"`
	VelYMmS     int16 `json:"velY_mm_s"`
	VelAngMradS int16 `json:"velAng_mrad_s"`

	Dribble uint8 `json:"dribble"`
	Kick    uint8 `json:"kick"`
	Chip    uint8 `json:"chip"`
	Info    uint8 `json:"info_bits"`
}
