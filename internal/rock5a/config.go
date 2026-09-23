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
// 仕様は ssl-Circuit の 2026/Firmware/MainBoard/MainBoard_V26_2/docs/SPI_PROTOCOL.md。
//
//	v1 (MainBoard_V26_1_1 まで): ヘッダ + 18 + フッタ = 20。12〜18 は 0 埋め
//	v2 (MainBoard_V26_2 から):   ヘッダ + 19 + フッタ = 21。12〜19 に IMU (加速度 XY・ヨーの角速度・姿勢角)
//
// 上り・下りとも同じ長さで揃える。下りの中身は 18 バイトのままで、v2 では 19 バイト目が予備 (0 を送る)。
type spiLayout struct {
	Name        string
	FrameSize   int
	PayloadSize int
	HasIMU      bool
	// VoltPerLSB は電圧のバイトの倍率 [V/LSB]。
	//
	// 旧: 0.1 V/LSB (×10)。**255 を超える 26 V 以上で一周して 0.4 V などになる不具合があった**
	// (uint8 のまま ×10 していた。実際に満充電の電池で 26 V → 4、27 V → 14 になった)。
	// 新 (V26_2 の修正版): 0.2 V/LSB (×5) で 51 V まで表せる。
	VoltPerLSB float64
}

var (
	spiLayoutV1 = spiLayout{Name: "v1 (20B, IMU なし)", FrameSize: 20, PayloadSize: 18, VoltPerLSB: 0.1}
	spiLayoutV2 = spiLayout{Name: "v2 (21B, IMU あり)", FrameSize: 21, PayloadSize: 19, HasIMU: true, VoltPerLSB: 0.2}
)

// IMU は機体に対して 90° 回して取り付けられている (2026-09-23 に実機で確認)。
//
//	IMU の +X = 機体の右   (機体の左側を持ち上げたら X が +5.4 m/s^2、Y はほぼ 0)
//	IMU の +Y = 機体の前   (ドリブラ側を持ち上げたら Y が -4.4 m/s^2、X はほぼ 0)
//
// こちらの約束は「前が +x、左が +y」なので、受け取った時点で直す。
// ヨーの角速度は面内の回転なので、この付け替えでは変わらない (反時計回りが正。1 周で +2π を確認)。
func bodyFromImuAccel(sensorX, sensorY float64) (forward, left float64) {
	return sensorY, -sensorX
}

// batteryVolts は電圧のバイトをボルトに直す。
//
// 21 バイトのフレームには、倍率を直す前の中間のファーム (×10 のまま IMU を載せた版) もある。
// こちらの電池は 14〜30 V なので、0.2 V/LSB で読んで 40 V を超えたら、その中間の版だと見なして
// 0.1 V/LSB で読み直す (どちらの版でも正しい値になる)。
func batteryVolts(l spiLayout, raw uint8) float64 {
	v := float64(raw) * l.VoltPerLSB
	if v > 40 {
		return float64(raw) * 0.1
	}
	return v
}

// IMU の生値 → SI の倍率 (SPI_PROTOCOL.md §2: 加速度 ×1000 [g]、ヨーの角速度 ×900 [rad/s]、姿勢角 ×10000 [rad])。
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
