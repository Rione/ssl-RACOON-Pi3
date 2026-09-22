package receive

import (
	"bytes"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"log"
	"net"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/state"
	"github.com/Rione/ssl-RACOON-Pi2/internal/util"
	"github.com/Rione/ssl-RACOON-Pi2/proto/pb_gen"
	"google.golang.org/protobuf/proto"
)

const directKickThreshold float32 = 100

var playBallDetectedSound func()

func SetPlayBallDetectedSound(fn func()) {
	playBallDetectedSound = fn
}

func RunClient(done <-chan struct{}, myID uint32, ip string) {
	// 全インターフェース(0.0.0.0)で待ち受ける。特定IPに固定バインドすると、
	// 実行中にロボットのIPが変わった場合(DHCP更新/Wi-Fi再接続/リンク断→再取得)に
	// 旧IPへ張り付いたままとなり、OFFER/OK_PC/DATA/KEEP_ALIVE を受信できなくなる。
	// (送信側 mw.RunServer はアンバインドで現在のIPから送るため、ロボットはDISCOVERを
	//  出し続けRAVENは新IPへOFFERを返すが、この受信ソケットが旧IPだと永久に届かず、
	//  ロボット再起動でしか復旧しなくなる。) IP変化に追従するため 0.0.0.0 で待ち受ける。
	serverAddr := &net.UDPAddr{
		IP:   net.IPv4zero,
		Port: state.UDPRecvPort,
	}

	serverConn, err := net.ListenUDP("udp", serverAddr)
	util.CheckError(err)
	defer serverConn.Close()
	log.Printf("[AI RX] Listening on 0.0.0.0:%d (startup local IP was %s)", state.UDPRecvPort, ip)

	buf := make([]byte, 1024)
	log.Printf("[AI RX] Started listening for PC on port %d...", state.UDPRecvPort)

	for {
		select {
		case <-done:
			return
		default:
			n, addr, err := serverConn.ReadFromUDP(buf)
			if err != nil {
				log.Printf("[AI RX] Error reading UDP message: %v", err)
				continue
			}
			if n == 0 {
				continue
			}

			header := buf[0]
			robotId := uint32((header >> 4) & 0x0F)
			cmdId := header & 0x0F

			if robotId != myID {
				continue
			}

			state.StateMu.Lock()

			switch cmdId {
			case 0x02: // OFFER
				state.PcAddress = pcReceiveAddr(addr)
				state.ConnectionState = state.StateOffered
				state.LastRecvTime.Store(time.Now())
				log.Printf("[AI RX] Received OFFER from %s. State -> OFFERED", state.PcAddress.String())

			case 0x04: // OK_PC
				if state.ConnectionState == state.StateOffered && isSamePcIP(addr) {
					state.ConnectionState = state.StateConnected
					state.LastRecvTime.Store(time.Now())
					log.Printf("[AI RX] Received OK_PC from %s. State -> CONNECTED", addr.IP.String())
				}

			case 0x06: // DATA (BotCmd)
				if state.ConnectionState != state.StateConnected || !isSamePcIP(addr) {
					break
				}

				state.LastRecvTime.Store(time.Now())
				state.LastCmdRecvTime.Store(time.Now())

				if n > 1 {
					packet := &pb_gen.GrSim_Packet{}
					if err := proto.Unmarshal(buf[1:n], packet); err != nil {
						log.Printf("Error unmarshaling DATA: %v", err)
					} else {
						processRobotCommands(packet, myID)
					}
				}

			case 0x07: // KEEP_ALIVE
				if state.ConnectionState != state.StateConnected || !isSamePcIP(addr) {
					break
				}

				state.LastRecvTime.Store(time.Now())
			}
			state.StateMu.Unlock()
		}
	}
}

func pcReceiveAddr(addr *net.UDPAddr) *net.UDPAddr {
	if addr == nil {
		return nil
	}
	return &net.UDPAddr{
		IP:   append(net.IP(nil), addr.IP...),
		Port: state.PCRecvPort,
		Zone: addr.Zone,
	}
}

func isSamePcIP(addr *net.UDPAddr) bool {
	return state.PcAddress != nil && addr != nil && state.PcAddress.IP.Equal(addr.IP)
}

func processRobotCommands(packet *pb_gen.GrSim_Packet, myID uint32) {
	robotCmds := packet.Commands.GetRobotCommands()

	if state.DebugReceive {
		log.Printf("[AI RX] Received packet with %d robot commands", len(robotCmds))
	}

	for _, cmd := range robotCmds {
		if cmd.GetId() != myID {
			continue
		}

		if state.DebugReceive {
			logReceivedCommand(cmd)
		}

		processCommand(cmd)
	}
}

func logReceivedCommand(cmd *pb_gen.GrSim_Robot_Command) {
	log.Printf("[AI RX] === Robot ID: %d (Match) ===", cmd.GetId())
	log.Printf("[AI RX] VelTangent: %.3f m/s, VelNormal: %.3f m/s, VelAngular: %.3f rad/s",
		cmd.GetVeltangent(), cmd.GetVelnormal(), cmd.GetVelangular())
	log.Printf("[AI RX] KickSpeedX: %.1f, KickSpeedZ: %.1f, Spinner: %t, Wheel1(DribblePower): %.1f",
		cmd.GetKickspeedx(), cmd.GetKickspeedz(), cmd.GetSpinner(), cmd.GetWheel1())
	fmt.Println("---")
}

func processCommand(cmd *pb_gen.GrSim_Robot_Command) {
	kickSpeedX := cmd.GetKickspeedx()
	kickSpeedZ := cmd.GetKickspeedz()

	if kickSpeedX >= directKickThreshold {
		state.DoDirectKick = true
		kickSpeedX -= directKickThreshold
	}
	if kickSpeedZ >= directKickThreshold {
		state.DoDirectChipKick = true
		kickSpeedZ -= directKickThreshold
	}

	velTangent := float64(cmd.GetVeltangent())
	velNormal := float64(cmd.GetVelnormal())
	velAngular := float64(cmd.GetVelangular())
	spinner := cmd.GetSpinner()

	if kickSpeedX > 0 || kickSpeedZ > 0 {
		log.Printf("ID: %d, KickX: %.2f, KickZ: %.2f, VelT: %.2f, VelN: %.2f, VelA: %.2f, Spinner: %t",
			cmd.GetId(), cmd.GetKickspeedx(), cmd.GetKickspeedz(), velTangent, velNormal, velAngular, spinner)
	}

	payload := state.SendPayload{
		VelX:   int16(velTangent * 1000),
		VelY:   int16(velNormal * 1000),
		VelAng: int16(velAngular * 1000),
	}

	if spinner {
		spinnerVel := cmd.GetWheel1()
		if spinnerVel > 100 {
			spinnerVel = 100
		} else if spinnerVel < 0 {
			spinnerVel = 0
		}
		payload.DribblePower = uint8(spinnerVel)
	}

	if kickSpeedX > 0 {
		state.KickerVal = uint8(kickSpeedX * 10)
		state.KickerEnable = true
	}
	if state.KickerEnable {
		payload.KickPower = state.KickerVal
	}

	if kickSpeedZ > 0 {
		state.ChipVal = uint8(kickSpeedZ * 10)
		state.ChipEnable = true
	}
	if state.ChipEnable {
		payload.ChipPower = state.ChipVal
	}

	payload.Informations &= ^uint8(state.InfoEmgStop)
	if state.DoDirectKick {
		payload.Informations |= state.InfoDirectKick
	}
	if state.DoDirectChipKick {
		payload.Informations |= state.InfoDirectChip
	}
	payload.Informations |= state.InfoDoCharge

	state.SetSendPayload(mustEncodeSendPayload(payload))
}

func mustEncodeSendPayload(payload state.SendPayload) []byte {
	var buf bytes.Buffer
	if err := binary.Write(&buf, binary.LittleEndian, payload); err != nil {
		log.Fatal(err)
	}
	return buf.Bytes()
}

func ReceiveData(done <-chan struct{}, myID uint32, ip string) {
	serverAddr := &net.UDPAddr{
		IP:   net.ParseIP(ip),
		Port: state.UDPCameraPort,
	}

	serverConn, err := net.ListenUDP("udp", serverAddr)
	util.CheckError(err)
	defer serverConn.Close()

	buf := make([]byte, 20240)

	for {
		select {
		case <-done:
			return
		default:
			n, _, _ := serverConn.ReadFromUDP(buf)

			jsonData := &state.ImageData{}
			if err := json.Unmarshal(buf[0:n], jsonData); err != nil {
				log.Printf("JSON unmarshal error: %v", err)
				continue
			}

			state.ApplyMissingBallCoords(jsonData)

			state.ImageDataPtr = jsonData
			state.ImageResponseData.Frame = jsonData.Frame

			if jsonData.IsBallExit && !state.PrevBallDetected {
				if state.DebugCamera && playBallDetectedSound != nil {
					go playBallDetectedSound()
				}
			}
			state.PrevBallDetected = jsonData.IsBallExit
		}
	}
}
