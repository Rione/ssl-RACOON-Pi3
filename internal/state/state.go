package state

import (
	"net"
	"sync"
	"sync/atomic"
	"time"
)

const (
	BatteryLowThreshold      = 140
	BatteryCriticalThreshold = 135

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
}

var (
	FlWheelSpeedRadS float32
	BlWheelSpeedRadS float32
	BrWheelSpeedRadS float32
	FrWheelSpeedRadS float32
)

var IsControlByRobotMode bool

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
	DebugSerial      bool = false
	DebugReceive     bool = false
	DebugCamera      bool = false
	DebugWheelGraph  bool = false
	DryRun       bool = false
	VelX1000     bool = false

	PowerShutdownMode bool = false

	// IsNewRobot is true when running on Rock5A (new robot) and false on
	// Raspberry Pi (pi4). Set by the board-specific registerPlatform.
	IsNewRobot bool = false

	// MACAddress is the hardware (MAC) address of the active NIC, formatted as
	// "aa:bb:cc:dd:ee:ff". Used by RAVEN to identify each robot's board for
	// motor individual-difference management. Empty when it could not be
	// resolved. Set once at startup alongside the local IP.
	MACAddress string = ""

	// Version is the RACOON-Pi2 build version (e.g. "v6.2.3"), sent to RAVEN in
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
