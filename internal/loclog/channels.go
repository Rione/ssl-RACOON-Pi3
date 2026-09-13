package loclog

// Channel は MCAP のトピック。計画 §6.2 の設計に対応する。
type Channel int

// チャンネル一覧。
const (
	// ChSPI は SPI の生フレーム (tx / rx)。
	//
	// STM のバイト配置が未確定でも、生の 20 バイトを記録しておけば後で
	// 再デコードできる。P1 を仕様確定より先に始められる根拠 (計画 §6.1 / §11)。
	ChSPI Channel = iota
	// ChVision は SSL_WrapperPacket の生バイト列 (protobuf)。
	ChVision
	// ChVisionMeta は vision パケットの時刻まわりの抽出。
	ChVisionMeta
	// ChWheel はデコード後の 4 輪角速度。
	ChWheel
	// ChIMU はデコード後のジャイロ・加速度。現状 STM が送ってこないので空。
	ChIMU
	// ChEstState は推定器の出力。
	ChEstState
	// ChEstInnovation は残差・正規化イノベーション・Huber 重み・棄却フラグ。
	ChEstInnovation
	// ChEstTiming は dt の実測、retrodiction 時間、破棄観測数、ループ処理時間。
	ChEstTiming
	// ChControlTarget は RAVEN から来た目標位置。
	ChControlTarget
	// ChOutCommand は STM へ送った指令。
	ChOutCommand
	// ChRecorderStatus は記録自体の健全性。
	//
	// 落ちたことがログの外にしか無いと、後日ログだけ見る人に穴が見えない。
	ChRecorderStatus

	numChannels
)

// channelDef はトピック名とエンコーディング。
type channelDef struct {
	topic    string
	encoding string
}

var channelDefs = [numChannels]channelDef{
	ChSPI:            {"/in/spi", "json"},
	ChVision:         {"/in/vision", "protobuf"},
	ChVisionMeta:     {"/in/vision_meta", "json"},
	ChWheel:          {"/sensors/wheel", "json"},
	ChIMU:            {"/sensors/imu", "json"},
	ChEstState:       {"/est/state", "json"},
	ChEstInnovation:  {"/est/innovation", "json"},
	ChEstTiming:      {"/est/timing", "json"},
	ChControlTarget:  {"/control/target", "json"},
	ChOutCommand:     {"/out/command", "json"},
	ChRecorderStatus: {"/recorder/status", "json"},
}

// Topic はチャンネルのトピック名を返す。
func (c Channel) Topic() string {
	if c < 0 || c >= numChannels {
		return "/unknown"
	}
	return channelDefs[c].topic
}

// Encoding はチャンネルのメッセージエンコーディングを返す。
func (c Channel) Encoding() string {
	if c < 0 || c >= numChannels {
		return "json"
	}
	return channelDefs[c].encoding
}

func (c Channel) String() string { return c.Topic() }
