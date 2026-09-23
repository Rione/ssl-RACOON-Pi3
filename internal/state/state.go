package state

import (
	"net"
	"sync"
	"sync/atomic"
	"time"
)

const (
	// 生の値で持っていた古い閾値 (0.1 V/LSB 前提)。判定は BatteryVolts と下の V 版で行う。
	BatteryLowThreshold      = 140
	BatteryCriticalThreshold = 135

	// 電圧の閾値 [V]。STM の送り方 (倍率) が世代で違うので、生の値ではなくボルトで比べる。
	BatteryLowVolts      = float64(BatteryLowThreshold) / 10
	BatteryCriticalVolts = float64(BatteryCriticalThreshold) / 10

	Port          = ":9191"
	UDPRecvPort   = 20011
	UDPCameraPort = 31133
	CalibPort     = 31134
	TunerPort     = 31135
	MulticastAddr = "224.5.69.4"
	MulticastPort = "16941"
	PCRecvPort    = 16941

	KickHoldDuration  = 500 * time.Millisecond
	NoRecvTimeout     = 1 * time.Second
	ChargeStopTimeout = 15 * time.Second

	SensorPhotoMask    = 0b00000001
	SensorDribblerMask = 0b00000010
	SensorNewDribMask  = 0b00000100

	InfoEmgStop        = 0b00000001
	InfoDirectKick     = 0b00000010
	InfoDirectChip     = 0b00000100
	InfoDoCharge       = 0b00010000
	InfoSignalReceived = 0b00100000
	InfoCtrlByRobot    = 0b01000000

	PowerCmdShutdown = 0x99
)

const (
	StateDiscovering = 0
	StateOffered     = 1
	StateConnected   = 2
)

// AtomicTime は time.Time を複数goroutineからロックなしで読み書きするためのラッパ。
type AtomicTime struct {
	nano atomic.Int64
}

func (a *AtomicTime) Store(t time.Time) { a.nano.Store(t.UnixNano()) }

func (a *AtomicTime) Since() time.Duration {
	return time.Duration(time.Now().UnixNano() - a.nano.Load())
}

var (
	sendPayloadMu sync.RWMutex
	sendPayload   []byte
)

func SetSendPayload(data []byte) {
	sendPayloadMu.Lock()
	sendPayload = append([]byte(nil), data...)
	sendPayloadMu.Unlock()
}

func GetSendPayload() []byte {
	sendPayloadMu.RLock()
	defer sendPayloadMu.RUnlock()
	if len(sendPayload) == 0 {
		return nil
	}
	return append([]byte(nil), sendPayload...)
}

type RecvData struct {
	Volt              uint8
	SensorInformation uint8
	CapPower          uint8
	FlWheelSpeed      int16
	BlWheelSpeed      int16
	BrWheelSpeed      int16
	FrWheelSpeed      int16
	Footer            uint8
	Reserved          uint8

	// IMU (MainBoard_V26_2 以降の 21 バイトのフレームだけ。HasIMU が false なら 0)。
	HasIMU      bool
	AccelXRaw   int16 // 1 LSB = 1 mg
	AccelYRaw   int16
	YawRateRaw  int16 // 900 LSB = 1 rad/s
	YawAngleRaw int16 // 10000 LSB = 1 rad (STM 側の Madgwick。参考値)
}

var (
	FlWheelSpeedRadS float32
	BlWheelSpeedRadS float32
	BrWheelSpeedRadS float32
	FrWheelSpeedRadS float32
)

// BatteryVolts は STM から届いた電圧 [V]。板ごとの倍率を当てはめた後の値で、
// 生の Recvdata.Volt ではなくこちらで判定する (SPI_PROTOCOL.md の倍率が世代で違うため)。
var BatteryVolts float64

// IMU の SI に直した値 (SPI の周期ごとに更新。IMU の無いファームでは 0 のまま)。
// ImuValid が false のときは中身を使わないこと。
var (
	ImuValid       bool
	ImuAccelXMS2   float64 // 機体座標の前後 [m/s^2]
	ImuAccelYMS2   float64 // 機体座標の左右 [m/s^2]
	ImuYawRateRadS float64 // ヨーの角速度 [rad/s] (STM 側でバイアスを引いたもの)
	ImuYawRad      float64 // STM 側の Madgwick の姿勢角 [rad] (参考。制御には使わない)
)

var IsControlByRobotMode bool

// TrajPoCActive は時刻つき軌道追従の PoC が走っている間 true。
// その間は PC からの DATA (速度・キック) を反映しない (docs/traj-poc.md)。
var TrajPoCActive atomic.Bool

// SPIRxValidAt は STM から最後に正しいフレームを受け取った時刻 (UnixNano)。0 なら一度も無い。
// PoC は STM が応答していないと走り出さない (車輪の値が 0 のまま更新されず、車輪と vision の検査も効かないため)。
var SPIRxValidAt atomic.Int64

type SendPayload struct {
	VelX          int16
	VelY          int16
	VelAng        int16
	DribblePower  uint8
	KickPower     uint8
	ChipPower     uint8
	RelativeX     int16
	RelativeY     int16
	RelativeTheta int16
	CameraBallX   uint8
	CameraBallY   uint8
	Informations  uint8
}

var (
	Recvdata RecvData
	ImuError bool = false
)

var (
	StateMu         sync.Mutex
	ConnectionState int = StateDiscovering
	PcAddress       *net.UDPAddr
	LastRecvTime    AtomicTime // OFFER/OK_PC/DATA/KEEP_ALIVE。接続生存・充電停止用
	LastCmdRecvTime AtomicTime // DATA(0x06)のみ。速度クリアのフェイルセーフ用
)

func init() {
	now := time.Now()
	LastRecvTime.Store(now)
	LastCmdRecvTime.Store(now)
}

var (
	IsRobotError      = false
	RobotErrorCode    = 0
	RobotErrorMessage = ""
)

var AlarmIgnore = false

var (
	KickerEnable     bool  = false
	KickerVal        uint8 = 0
	ChipEnable       bool  = false
	ChipVal          uint8 = 0
	DoDirectChipKick bool  = false
	DoDirectKick     bool  = false
)

// BallCoordMissing is sent as ball_camera_x/y when no ball is detected.
const BallCoordMissing float32 = 9999

// ApplyMissingBallCoords sets x/y to BallCoordMissing when no ball is detected.
func ApplyMissingBallCoords(data *ImageData) {
	if data == nil || data.IsBallExit {
		return
	}
	data.ImageX = BallCoordMissing
	data.ImageY = BallCoordMissing
}

type ImageData struct {
	IsBallExit  bool    `json:"isball"`
	ImageX      float32 `json:"x"`
	ImageY      float32 `json:"y"`
	Frame       string  `json:"frame"`
	FrameWidth  int     `json:"frameWidth"`
	FrameHeight int     `json:"frameHeight"`
}

var ImageDataPtr *ImageData
var PrevBallDetected bool

type ImageResponse struct {
	Frame string `json:"frame"`
}

var ImageResponseData ImageResponse

var (
	DebugSerial     bool = false
	DebugReceive    bool = false
	DebugCamera     bool = false
	DebugWheelGraph bool = false
	DryRun          bool = false
	VelX1000        bool = false

	PowerShutdownMode bool = false

	// IsNewRobot is true when running on Rock5A (new robot) and false on
	// Raspberry Pi (pi4). Set by the board-specific registerPlatform.
	IsNewRobot bool = false

	// MACAddress is the hardware (MAC) address of the active NIC, formatted as
	// "aa:bb:cc:dd:ee:ff". Used by RAVEN to identify each robot's board for
	// motor individual-difference management. Empty when it could not be
	// resolved. Set once at startup alongside the local IP.
	MACAddress string = ""

	// Version is the RACOON-Pi3 build version (e.g. "v6.2.3"), sent to RAVEN in
	// PiToMw and shown in the Robot Status pane. On dev builds it may be
	// "(devel)"/"unknown". Set once at startup from upgrade.GetVersion().
	Version string = ""
)

type Adjustment struct {
	MinThreshold         string  `json:"minThreshold"`
	MaxThreshold         string  `json:"maxThreshold"`
	BallDetectRadius     int     `json:"ballDetectRadius"`
	CircularityThreshold float32 `json:"circularityThreshold"`
}

var DefaultAdjustment = Adjustment{
	MinThreshold:         "1, 120, 100",
	MaxThreshold:         "15, 255, 255",
	BallDetectRadius:     150,
	CircularityThreshold: 0.2,
}

// 自己位置推定の計測基盤 (docs/self-localization-plan.md の P1) の設定。
var (
	// LocLogDir は計測ログ(MCAP)の出力先。空なら記録しない。
	LocLogDir string
	// LocProfile は STM フレームのプロファイル名か JSON パス。
	LocProfile string
	// LocTeam は自機のチーム色 ("blue" / "yellow")。
	//
	// SSL-Vision は robots_blue / robots_yellow を分けて送り、robot_id は
	// チーム内で採番されるので、色が分からないと自機を特定できない。
	// 現状ロボットは自分の色を知る経路を持たないので、起動フラグで与える。
	LocTeam string
	// LocVisionAddr は SSL-Vision のマルチキャスト。空なら既定値。
	LocVisionAddr string
	// LocVisionIface は受信に使う NIC 名。空ならシステム既定。
	LocVisionIface string
	// LocIdent は機体パラメータ同定の加振を実行するか。ロボットが自走する。
	LocIdent bool
)

// localizationShutdown は計測ログを閉じる後始末。
// SIGINT で落とすときに、書き残しでログの最後が消えるのを防ぐ。
var localizationShutdown atomic.Pointer[func()]

// SetLocalizationShutdown は後始末を登録する。
func SetLocalizationShutdown(fn func()) {
	if fn == nil {
		localizationShutdown.Store(nil)
		return
	}
	localizationShutdown.Store(&fn)
}

// RunLocalizationShutdown は登録された後始末を 1 度だけ実行する。
func RunLocalizationShutdown() {
	if fn := localizationShutdown.Swap(nil); fn != nil {
		(*fn)()
	}
}
