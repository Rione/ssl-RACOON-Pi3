package api

import (
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/link"
	"github.com/Rione/ssl-RACOON-Pi2/internal/mw"
	"github.com/Rione/ssl-RACOON-Pi2/internal/state"
)

var (
	pythonCmd   *exec.Cmd
	robotID     uint32
)

func Run(done <-chan struct{}, myID uint32) {
	robotID = myID
	if err := restartPythonProcess(); err != nil {
		log.Printf("Pythonプロセス開始エラー（プログラムは継続します）: %v", err)
	}

	listener, err := net.Listen("tcp", state.Port)
	if err != nil {
		log.Fatal(err)
	}
	defer listener.Close()

	log.Printf("API Server listening on %s", state.Port)

	for {
		conn, err := listener.Accept()
		if err != nil {
			log.Printf("Accept error: %v", err)
			continue
		}
		log.Println("Remote API Connected by", conn.RemoteAddr())
		go handleRequest(conn)
	}
}

func sendHTTPResponse(conn net.Conn, statusCode int, contentType string, body string) {
	statusText := map[int]string{
		200: "OK",
		400: "Bad Request",
		500: "Internal Server Error",
	}
	fmt.Fprintf(conn, "HTTP/1.1 %d %s\r\n", statusCode, statusText[statusCode])
	fmt.Fprintf(conn, "Content-Type: %s; charset=utf-8\r\n", contentType)
	fmt.Fprintf(conn, "Content-Length: %d\r\n", len(body))
	fmt.Fprintf(conn, "\r\n")
	fmt.Fprint(conn, body)
}

func sendErrorResponse(conn net.Conn, statusCode int) {
	statusText := map[int]string{
		400: "Bad Request",
		500: "Internal Server Error",
	}
	body := fmt.Sprintf("%d %s\r\n", statusCode, statusText[statusCode])
	sendHTTPResponse(conn, statusCode, "text/plain", body)
}

func handleRequest(conn net.Conn) {
	defer conn.Close()

	buf := make([]byte, 1024)
	if _, err := conn.Read(buf); err != nil {
		log.Printf("Read error: %v", err)
		return
	}

	request := string(buf)
	parts := strings.Split(request, " ")
	if len(parts) < 2 {
		sendErrorResponse(conn, 400)
		return
	}

	method := parts[0]
	path := parts[1]

	if method != "GET" {
		sendErrorResponse(conn, 400)
		return
	}

	pathParts := strings.Split(path, "/")
	if len(pathParts) < 2 {
		sendErrorResponse(conn, 400)
		return
	}

	endpoint := pathParts[1]

	switch endpoint {
	case "buzzer":
		handleBuzzer(conn, pathParts)
	case "ignorebatterylow":
		handleIgnoreBatteryLow(conn)
	case "image":
		handleImage(conn)
	case "updatepython":
		handleUpdatePython(conn)
	case "changeadjustment":
		handleChangeAdjustment(conn, pathParts)
	case "calibballcolor":
		handleCalibBallColor(conn)
	case "color-tuner":
		handleColorTunerPage(conn)
	case "colorpreview":
		handleColorPreview(conn)
	case "colorthresholds":
		handleColorThresholds(conn)
	case "setcolor":
		handleSetColor(conn, pathParts)
	case "relaxcolor":
		handleRelaxColor(conn, pathParts)
	case "powershutdown":
		handlePowerShutdown(conn)
	default:
		handleStatus(conn)
	}
}

func handleBuzzer(conn net.Conn, pathParts []string) {
	if len(pathParts) < 5 {
		sendErrorResponse(conn, 400)
		return
	}

	tone, err := strconv.Atoi(pathParts[3])
	if err != nil || tone < 0 || tone > 99 {
		sendErrorResponse(conn, 400)
		return
	}

	duration, err := strconv.Atoi(pathParts[4])
	if err != nil || duration < 50 || duration > 3000 {
		sendErrorResponse(conn, 400)
		return
	}

	sendHTTPResponse(conn, 200, "text/plain", "BUZZER OK\r\n")
	link.RingBuzzerSync(tone, time.Duration(duration)*time.Millisecond, 0)
}

func handleIgnoreBatteryLow(conn net.Conn) {
	state.AlarmIgnore = true
	sendHTTPResponse(conn, 200, "text/plain", "IGNORE BATTERY LOW OK\r\n")
}

func handlePowerShutdown(conn net.Conn) {
	if !state.PowerShutdownMode {
		log.Println("Power shutdown mode requested via API")
	}
	state.PowerShutdownMode = true
	sendHTTPResponse(conn, 200, "text/plain", "POWER SHUTDOWN OK\r\n")
}

func handleImage(conn net.Conn) {
	response, err := json.Marshal(state.ImageResponseData.Frame)
	if err != nil {
		sendErrorResponse(conn, 500)
		return
	}
	sendHTTPResponse(conn, 200, "application/json", string(response))
}

func handleUpdatePython(conn net.Conn) {
	sendHTTPResponse(conn, 200, "text/plain", "UPDATE PYTHON OK\r\n")
	if err := restartPythonProcess(); err != nil {
		log.Printf("Pythonプロセス再起動エラー: %v", err)
	}
}

func handleChangeAdjustment(conn net.Conn, pathParts []string) {
	if len(pathParts) < 6 {
		sendErrorResponse(conn, 400)
		return
	}

	minThreshold := pathParts[2]
	maxThreshold := pathParts[3]

	ballDetectRadius, err := strconv.Atoi(pathParts[4])
	if err != nil {
		sendErrorResponse(conn, 400)
		return
	}

	circularityThreshold, err := strconv.ParseFloat(pathParts[5], 32)
	if err != nil {
		sendErrorResponse(conn, 400)
		return
	}

	data := state.Adjustment{
		MinThreshold:         minThreshold,
		MaxThreshold:         maxThreshold,
		BallDetectRadius:     ballDetectRadius,
		CircularityThreshold: float32(circularityThreshold),
	}

	if err := mw.SaveAdjustmentConfig(data); err != nil {
		log.Printf("しきい値保存エラー: %v", err)
		sendErrorResponse(conn, 500)
		return
	}

	mw.ReloadAdjustment()

	sendHTTPResponse(conn, 200, "text/plain", "CHANGE ADJUSTMENT OK\r\n")

	if err := restartPythonProcess(); err != nil {
		log.Printf("Pythonプロセス再起動エラー: %v", err)
	}
}

// handleCalibBallColor triggers a one-shot YOLO calibration in the camera
// process, which recomputes HSV thresholds from a detected ball and writes
// threshold.json. On success the MW threshold cache is reloaded so the new
// values take effect without restarting the camera.
func handleCalibBallColor(conn net.Conn) {
	body, ok, err := requestCalibration()
	if err != nil {
		log.Printf("キャリブレーション要求エラー: %v", err)
		sendErrorResponse(conn, 500)
		return
	}

	if !ok {
		sendHTTPResponse(conn, 400, "application/json", string(body))
		return
	}

	mw.ReloadAdjustment()
	sendHTTPResponse(conn, 200, "application/json", string(body))
}

// requestCalibration asks the camera process (TCP, localhost) to run a
// calibration and returns the raw JSON response, whether it reported success,
// and any transport error.
func requestCalibration() ([]byte, bool, error) {
	addr := fmt.Sprintf("127.0.0.1:%d", state.CalibPort)
	conn, err := net.DialTimeout("tcp", addr, calibDialTimeout)
	if err != nil {
		return nil, false, fmt.Errorf("カメラプロセスへ接続できませんでした: %w", err)
	}
	defer conn.Close()

	if err := conn.SetDeadline(time.Now().Add(calibTimeout)); err != nil {
		return nil, false, err
	}

	if _, err := conn.Write([]byte("calib\n")); err != nil {
		return nil, false, fmt.Errorf("キャリブレーション要求の送信に失敗しました: %w", err)
	}

	data, err := io.ReadAll(conn)
	if err != nil {
		return nil, false, fmt.Errorf("キャリブレーション結果の受信に失敗しました: %w", err)
	}

	var parsed struct {
		OK bool `json:"ok"`
	}
	if err := json.Unmarshal(data, &parsed); err != nil {
		return data, false, fmt.Errorf("キャリブレーション結果の解析に失敗しました: %w", err)
	}

	return data, parsed.OK, nil
}

type statusBallResponse struct {
	Detected bool    `json:"detected"`
	CameraX  float32 `json:"cameraX"`
	CameraY  float32 `json:"cameraY"`
}

type statusWheelSpeedMS struct {
	FL float32 `json:"fl"`
	BL float32 `json:"bl"`
	BR float32 `json:"br"`
	FR float32 `json:"fr"`
}

type statusWheelSpeedRaw struct {
	FL int16 `json:"fl"`
	BL int16 `json:"bl"`
	BR int16 `json:"br"`
	FR int16 `json:"fr"`
}

type statusResponse struct {
	RobotID                 uint32              `json:"robotId"`
	ConnectionState         string              `json:"connectionState"`
	IsNewRobot              bool                `json:"isNewRobot"`
	Volt                    float32             `json:"VOLT"`
	IsDetectPhotoSensor     bool                `json:"ISDETECTPHOTOSENSOR"`
	IsDetectDribblerSensor  bool                `json:"ISDETECTDRIBBLERSENSOR"`
	IsNewDribbler           bool                `json:"ISNEWDRIBBLER"`
	CapPower                uint8               `json:"capPower"`
	WheelSpeedMS            statusWheelSpeedMS  `json:"wheelSpeedMS"`
	WheelSpeedRaw           statusWheelSpeedRaw `json:"wheelSpeedRaw"`
	Ball                    statusBallResponse  `json:"ball"`
	Thresholds              state.Adjustment    `json:"thresholds"`
	Error                   bool                `json:"ERROR"`
	ErrorCode               int                 `json:"ERRORCODE"`
	ErrorMessage            string              `json:"ERRORMESSAGE"`
}

func connectionStateName(s int) string {
	switch s {
	case state.StateOffered:
		return "offered"
	case state.StateConnected:
		return "connected"
	default:
		return "discovering"
	}
}

func buildStatusResponse() statusResponse {
	detectPhotoSensor := state.Recvdata.SensorInformation&state.SensorPhotoMask != 0
	detectDribblerSensor := state.Recvdata.SensorInformation&state.SensorDribblerMask != 0
	isNewDribbler := state.Recvdata.SensorInformation&state.SensorNewDribMask != 0

	var isBallDetected bool
	var imageX, imageY float32 = state.BallCoordMissing, state.BallCoordMissing
	if state.ImageDataPtr != nil {
		isBallDetected = state.ImageDataPtr.IsBallExit
		imageX = state.ImageDataPtr.ImageX
		imageY = state.ImageDataPtr.ImageY
		if !isBallDetected {
			imageX = state.BallCoordMissing
			imageY = state.BallCoordMissing
		}
	}

	state.StateMu.Lock()
	connState := state.ConnectionState
	state.StateMu.Unlock()

	return statusResponse{
		RobotID:                robotID,
		ConnectionState:        connectionStateName(connState),
		IsNewRobot:             state.IsNewRobot,
		Volt:                   float32(state.Recvdata.Volt) / 10.0,
		IsDetectPhotoSensor:    detectPhotoSensor,
		IsDetectDribblerSensor: detectDribblerSensor,
		IsNewDribbler:          isNewDribbler,
		CapPower:               state.Recvdata.CapPower,
		WheelSpeedMS: statusWheelSpeedMS{
			FL: state.FlWheelSpeedRadS,
			BL: state.BlWheelSpeedRadS,
			BR: state.BrWheelSpeedRadS,
			FR: state.FrWheelSpeedRadS,
		},
		WheelSpeedRaw: statusWheelSpeedRaw{
			FL: state.Recvdata.FlWheelSpeed,
			BL: state.Recvdata.BlWheelSpeed,
			BR: state.Recvdata.BrWheelSpeed,
			FR: state.Recvdata.FrWheelSpeed,
		},
		Ball: statusBallResponse{
			Detected: isBallDetected,
			CameraX:  imageX,
			CameraY:  imageY,
		},
		Thresholds:   mw.GetAdjustment(),
		Error:        state.IsRobotError,
		ErrorCode:    state.RobotErrorCode,
		ErrorMessage: state.RobotErrorMessage,
	}
}

func handleStatus(conn net.Conn) {
	response, err := json.Marshal(buildStatusResponse())
	if err != nil {
		sendErrorResponse(conn, 500)
		return
	}
	sendHTTPResponse(conn, 200, "application/json", string(response))
}

const (
	calibDialTimeout = 5 * time.Second
	// Generous: the first calibration loads the YOLO model, which can take
	// several seconds on the robot.
	calibTimeout = 60 * time.Second
)

// cameraWorkDir returns the directory the camera process should run in. The
// camera package and threshold.json live next to the binary, so the binary's
// directory is used (falling back to the current working directory).
func cameraWorkDir() string {
	exe, err := os.Executable()
	if err != nil {
		return ""
	}
	return filepath.Dir(exe)
}

func restartPythonProcess() error {
	stopPythonProcess()

	log.Printf("Pythonプロセスを開始します（board=%s）。", cameraBoard)
	cmd := exec.Command("python3", "-m", "camera")
	configurePythonCmd(cmd)
	cmd.Env = append(os.Environ(), "RACOON_BOARD="+cameraBoard)
	if state.DebugCamera {
		cmd.Env = append(cmd.Env, "RACOON_CAMERA_DEBUG=1")
	}
	if dir := cameraWorkDir(); dir != "" {
		cmd.Dir = dir
		cmd.Env = append(cmd.Env, "PYTHONPATH="+dir)
	}
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr

	pythonCmd = cmd
	if err := pythonCmd.Start(); err != nil {
		return fmt.Errorf("Pythonプロセス開始エラー: %w", err)
	}

	log.Println("Pythonプロセスが正常に開始されました。")
	return nil
}

// StopPythonProcess terminates the camera Python process and any stale copies
// still holding the MIPI camera (libcamera allows only one client).
func StopPythonProcess() {
	stopPythonProcess()
}

func stopPythonProcess() {
	if pythonCmd != nil && pythonCmd.Process != nil {
		log.Println("既存のPythonプロセスを停止します。")
		_ = pythonCmd.Process.Kill()
		_, _ = pythonCmd.Process.Wait()
		pythonCmd = nil
	}

	// Orphans survive Ctrl+C of the Go binary; clear them before reopening the camera.
	_ = exec.Command("pkill", "-TERM", "-f", "python3 -m camera").Run()
	time.Sleep(300 * time.Millisecond)
	_ = exec.Command("pkill", "-KILL", "-f", "python3 -m camera").Run()
	time.Sleep(200 * time.Millisecond)
}
