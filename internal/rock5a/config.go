//go:build rock5a

package rock5a

const DefaultHostname = "DietPi\n"

const (
	SPIDevPath     = "/dev/spidev4.0"
	SPISpeedHz     = 1_000_000
	SPIFrameSize   = 20
	SPIPayloadSize = 18
	SPIRecvSize    = 11
	SPIFrameHeader = 0xFF
	SPIFrameFooter = 0xAA
	SPIPeriodMs    = 8
	WheelDiameterMm = 60.0
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
