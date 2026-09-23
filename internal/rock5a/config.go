//go:build rock5a

package rock5a

const DefaultHostname = "DietPi\n"

const (
	SPIDevPath = "/dev/spidev4.0"
	SPISpeedHz = 1_000_000
	// SPIRecvSize は STM から届く「車輪まで」の長さ [byte] (電圧・状態・キャパシタ + 4 輪)。
	SPIRecvSize     = 11
	SPIFrameHeader  = 0xFF
	SPIFrameFooter  = 0xAA
	SPIPeriodMs     = 8
	WheelDiameterMm = 60.0

	// MaxSPIFrameSize はどのファームでも収まる大きさ (バッファの確保用)。
	MaxSPIFrameSize = 21
)

// STM のフレームの形。ファームの世代で長さが違うので、通信しながら見分ける (spi.go)。
//
//	v1 (MainBoard_V26_1_1 まで): 21 バイト未満。ヘッダ + 18 + フッタ = 20。12〜18 は 0 埋め
//	v2 (MainBoard_V26_2 から):   ヘッダ + 19 + フッタ = 21。12〜19 に IMU (加速度 XY・ヨーの角速度・姿勢角)
type spiLayout struct {
	Name        string
	FrameSize   int
	PayloadSize int
	HasIMU      bool
}

var (
	spiLayoutV1 = spiLayout{Name: "v1 (20B, IMU なし)", FrameSize: 20, PayloadSize: 18}
	spiLayoutV2 = spiLayout{Name: "v2 (21B, IMU あり)", FrameSize: 21, PayloadSize: 19, HasIMU: true}
)

// IMU の生値 → SI の倍率 (ssl-Circuit の MainBoard_V26_2 src/unit/robot.c と同じ)。
const (
	spiAccelPerLSBG     = 0.001       // [g]     1 LSB = 1 mg
	spiYawRatePerLSBRad = 1.0 / 900.0 // [rad/s]
	spiYawPerLSBRad     = 0.0001      // [rad]   Madgwick の姿勢 (こちらでは使わない)
	gravityMS2          = 9.80665     // [m/s^2]
)

const (
	PIN_LED1_BANK = 4
	PIN_LED1_PORT = 0
	PIN_LED1_PIN  = 1
	PIN_LED2_BANK = 4
	PIN_LED2_PORT = 1
	PIN_LED2_PIN  = 2

	PIN_BUTTON1_BANK = 4
	PIN_BUTTON1_PORT = 1
	PIN_BUTTON1_PIN  = 4
	PIN_BUTTON2_BANK = 1
	PIN_BUTTON2_PORT = 1
	PIN_BUTTON2_PIN  = 0

	PIN_DIP1_BANK = 1
	PIN_DIP1_PORT = 1
	PIN_DIP1_PIN  = 3
	PIN_DIP2_BANK = 1
	PIN_DIP2_PORT = 1
	PIN_DIP2_PIN  = 2
	PIN_DIP3_BANK = 1
	PIN_DIP3_PORT = 1
	PIN_DIP3_PIN  = 1
	PIN_DIP4_BANK = 1
	PIN_DIP4_PORT = 1
	PIN_DIP4_PIN  = 5
)

const (
	PWMChipPath = "/sys/class/pwm/pwmchip1"
	PWMChannel  = 0
)
