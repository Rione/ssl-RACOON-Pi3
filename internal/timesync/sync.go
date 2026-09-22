// Package timesync は SSL-Vision PC と Rock5A のクロックを結びつける。
//
// この系の精度を決めるのは、フィルタの洗練ではなく「どの値がいつの瞬間のものか」
// を正しく扱えるかである。3 m/s のロボットにとって 10 ms のズレは 3 cm の
// 位置誤差になる (計画 §5)。
//
// クロック推定は姿勢推定フィルタとは別のフィルタにする。クロックのドリフトは
// 分オーダー、姿勢は ms オーダーで時定数が 4-5 桁違うので、同じ状態ベクトルに
// 混ぜると数値的に条件が悪くなり、姿勢側の共分散が意味を失う (計画 §5.3)。
//
// このパッケージにビルドタグは付けない。開発 PC でそのままテストできる。
package timesync

import (
	"time"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// Quality は写像の信頼度。
type Quality struct {
	// Valid は写像が使えるか。false なら呼び出し側は到着時刻で代用する。
	Valid bool
	// SkewPpm は推定したクロック周波数差 [ppm]。
	//
	// 水晶の確度は ±20-100 ppm。50 ppm なら 1 分で 3 ms、10 分の試合で 30 ms
	// ずれるので、オフセットだけでなくここまで扱う必要がある (計画 §5.1)。
	SkewPpm float64
	// OffsetNs は推定したオフセット + 最小片方向遅延 [ns]。
	//
	// 片方向観測だけでは「クロックオフセット」と「最小片方向遅延」は
	// 原理的に分離できない。分離できるのはスキューと遅延の変動分だけで、
	// 残る定数分はオフラインで実測して定数として持つ (計画 §5.3)。
	OffsetNs int64
	// Samples は推定に使った観測数。
	Samples int
	// Frozen はスキュー推定の暴走を検出し、直前の値を保持している状態か。
	//
	// 写像自体は使えるが、既に古い。上位は警報を出す判断に使う (計画 §5.5)。
	Frozen bool
}

// Sync は remote (vision PC) の時刻を Rock5A の時間軸へ写す。
type Sync interface {
	// Observe は 1 つの観測を取り込む。
	//   remote:  パケットに載っていた送信側時刻 (t_capture)
	//   arrival: Rock5A が受信した単調時刻
	Observe(remote, arrival localization.Stamp)

	// ToLocal は remote 時刻を Rock5A の時間軸へ写す。
	// Quality.Valid が false のとき、返る Stamp は arrival そのものになる。
	ToLocal(remote, arrival localization.Stamp) (localization.Stamp, Quality)

	// Reset は推定状態を捨てる。AP ローミングなどで経路が変わったときに呼ぶ。
	Reset()
}

// SecondsToStamp は vision の t_capture (秒, double) を Stamp へ変換する。
//
// 変換先は vision PC のクロック上の時刻であって、Rock5A の時間軸ではない。
// Rock5A へ写すには Sync.ToLocal を通すこと。
func SecondsToStamp(sec float64) localization.Stamp {
	return localization.Stamp(sec * float64(time.Second))
}

// Arrival は「送信側の時刻を一切信用せず、到着時刻をそのまま使う」写像。
//
// P1 (計測フェーズ) の既定。生の t_capture / t_sent / 到着時刻はすべて MCAP に
// 記録されるので、凸包法によるスキュー・オフセット推定 (P2) はログから
// オフラインで開発・検証してから実機へ入れられる。
//
// この実装を使っている間、vision 観測には片道遅延ぶんの系統誤差が丸ごと残る。
// 2 m/s で 20 ms の遅延なら 40 mm。P2 までは精度目標を評価できない。
type Arrival struct{}

// Observe は何もしない。
func (Arrival) Observe(remote, arrival localization.Stamp) {}

// ToLocal は到着時刻をそのまま返す。
func (Arrival) ToLocal(remote, arrival localization.Stamp) (localization.Stamp, Quality) {
	return arrival, Quality{Valid: false}
}

// Reset は何もしない。
func (Arrival) Reset() {}
