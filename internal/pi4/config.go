//go:build pi4

package pi4

const DefaultHostname = "raspberrypi\n"

const (
	Baudrate         = 230400
	SerialPortName   = "/dev/serial0"
	WheelDiameterMm  = 54.0
	PIN_LED1         = 18
	PIN_LED2         = 27
	PIN_BUZZER       = 13
	PIN_BUTTON1      = 22
	PIN_BUTTON2      = 24
	PIN_DIP1         = 4
	PIN_DIP2         = 5
	PIN_DIP3         = 6
	PIN_DIP4         = 25
)
