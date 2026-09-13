package stmframe

import (
	"encoding/binary"
	"errors"
	"fmt"
	"math"
	"time"
)

// ErrNoFrame はウィンドウ内に有効なフレームが 1 つも無いことを示す。
var ErrNoFrame = errors.New("stmframe: no valid frame in window")

// Decoder はプロファイルを 1 度だけ解釈し、以降アロケートせずにデコードする。
type Decoder struct {
	profile *Profile
	index   map[string]int // フィールド名 -> Values 内の位置
}

// NewDecoder はプロファイルからデコーダを構築する。
func NewDecoder(p *Profile) (*Decoder, error) {
	if p == nil {
		return nil, errors.New("stmframe: nil profile")
	}
	if err := p.Validate(); err != nil {
		return nil, err
	}
	d := &Decoder{profile: p, index: make(map[string]int, len(p.Fields))}
	for i := range p.Fields {
		d.index[p.Fields[i].Name] = i
	}
	return d, nil
}

// Profile は構築に使ったプロファイルを返す。
func (d *Decoder) Profile() *Profile { return d.profile }

// FrameSize はフレーム 1 個のバイト数を返す。
func (d *Decoder) FrameSize() int { return d.profile.FrameSize }

// FieldIndex は名前からフィールドの位置を引く。起動時に 1 度だけ呼び、
// 以降ホットパスでは Values.At(idx) を使うことでマップ参照を避ける。
func (d *Decoder) FieldIndex(name string) (int, bool) {
	i, ok := d.index[name]
	return i, ok
}

// Field は位置からフィールド定義を返す。
func (d *Decoder) Field(i int) *Field { return &d.profile.Fields[i] }

// NumFields はフィールド数を返す。
func (d *Decoder) NumFields() int { return len(d.profile.Fields) }

// Values は 1 フレーム分のデコード結果。NewValues で 1 度だけ確保して使い回す。
type Values struct {
	value   []float64
	raw     []int64
	present []bool
}

// NewValues はこのデコーダ用の結果バッファを確保する。
func (d *Decoder) NewValues() *Values {
	n := len(d.profile.Fields)
	return &Values{
		value:   make([]float64, n),
		raw:     make([]int64, n),
		present: make([]bool, n),
	}
}

// At は位置 i の物理量を返す。
func (v *Values) At(i int) float64 { return v.value[i] }

// RawAt は位置 i のスケール適用前の生値を返す。同定やログに使う。
// f32 型の場合は未定義。
func (v *Values) RawAt(i int) int64 { return v.raw[i] }

// Present は位置 i のフィールドがこのフレームから読めたかを返す。
func (v *Values) Present(i int) bool { return v.present[i] }

// Match は窓の中で見つかったフレーム。
type Match struct {
	// Offset は採用したフレームの窓内での先頭位置。
	Offset int
	// Count は窓内で有効と判定できたフレームの総数。
	//
	// 2 以上のとき、採用したフレームがどの SPI トランザクションに対応するかが
	// 一意に決まらない。1 フレームぶん (8 ms) 時刻がずれ得るので、
	// レイテンシを測る局面 (計画 §5.4 の tau_stm) ではこの値を必ず見ること。
	Count int
}

// Ambiguous は窓内に複数の有効フレームがあったかを返す。
func (m Match) Ambiguous() bool { return m.Count > 1 }

// Validate は buf の offset 位置がプロファイル通りのフレームかを調べる。
//
// ヘッダ・フッタに加えて、MustBeZero の予約領域も検査する。現行フレームでは
// これがフレーム同期の判別力の大半を担っている (Profile.SyncStrength 参照)。
func (d *Decoder) Validate(buf []byte, offset int) error {
	p := d.profile
	if offset < 0 || offset+p.FrameSize > len(buf) {
		return fmt.Errorf("stmframe: frame out of range at offset %d (buffer %d bytes)", offset, len(buf))
	}
	if p.Header != nil && buf[offset] != byte(*p.Header) {
		return fmt.Errorf("stmframe: header: expected %02x, got %02x", byte(*p.Header), buf[offset])
	}
	if p.Footer != nil && buf[offset+p.FrameSize-1] != byte(*p.Footer) {
		return fmt.Errorf("stmframe: footer: expected %02x, got %02x",
			byte(*p.Footer), buf[offset+p.FrameSize-1])
	}
	for _, r := range p.Reserved {
		if !r.MustBeZero {
			continue
		}
		for i := r.Offset; i < r.Offset+r.Length; i++ {
			if buf[offset+i] != 0 {
				return fmt.Errorf("stmframe: reserved[%d]: expected 00, got %02x", i, buf[offset+i])
			}
		}
	}
	return nil
}

// validAt は Validate と同じ判定をエラーを組み立てずに行う。
//
// Find は窓の全位置を総当たりするので、外れるたびに fmt.Errorf を作ると
// 1 周期あたり数十回のアロケーションになる。ホットパスはこちらを通す。
func (d *Decoder) validAt(buf []byte, offset int) bool {
	p := d.profile
	if offset < 0 || offset+p.FrameSize > len(buf) {
		return false
	}
	if p.Header != nil && buf[offset] != byte(*p.Header) {
		return false
	}
	if p.Footer != nil && buf[offset+p.FrameSize-1] != byte(*p.Footer) {
		return false
	}
	for _, r := range p.Reserved {
		if !r.MustBeZero {
			continue
		}
		for i := offset + r.Offset; i < offset+r.Offset+r.Length; i++ {
			if buf[i] != 0 {
				return false
			}
		}
	}
	return true
}

// Find は窓の中を走査して、最後に現れた有効フレームを返す。
//
// SPI は位相がずれる前提で運用されており、既存実装 (internal/rock5a/frame.go の
// findSPIFrame) もフレーム 2 個ぶんの窓を総当たりしている。その挙動をここへ移した。
// 「最後」を採るのは、最新のデータを使うため。
func (d *Decoder) Find(window []byte) (Match, error) {
	m := Match{Offset: -1}
	for i := 0; i+d.profile.FrameSize <= len(window); i++ {
		if d.validAt(window, i) {
			m.Offset = i
			m.Count++
		}
	}
	if m.Count == 0 {
		return Match{Offset: -1}, fmt.Errorf("%w (%d bytes)", ErrNoFrame, len(window))
	}
	return m, nil
}

// Decode は offset 位置のフレームを out へ展開する。アロケートしない。
//
// 呼び出し前に Validate / Find でフレーム位置が確定していること。
func (d *Decoder) Decode(buf []byte, offset int, out *Values) error {
	p := d.profile
	if offset < 0 || offset+p.FrameSize > len(buf) {
		return fmt.Errorf("stmframe: frame out of range at offset %d (buffer %d bytes)", offset, len(buf))
	}
	if len(out.value) != len(p.Fields) {
		return fmt.Errorf("stmframe: values buffer has %d slots, profile has %d fields",
			len(out.value), len(p.Fields))
	}
	for i := range p.Fields {
		f := &p.Fields[i]
		b := buf[offset+f.Offset:]
		raw, fv, isFloat := readField(f.Type, b)
		if isFloat {
			out.value[i] = fv*f.Scale + f.Bias
			out.raw[i] = 0
		} else {
			out.value[i] = float64(raw)*f.Scale + f.Bias
			out.raw[i] = raw
		}
		out.present[i] = true
	}
	return nil
}

// readField は型に応じて生値を取り出す。float 型のときだけ third が true になる。
func readField(t FieldType, b []byte) (raw int64, f float64, isFloat bool) {
	switch t {
	case TypeU8:
		return int64(b[0]), 0, false
	case TypeI8:
		return int64(int8(b[0])), 0, false
	case TypeU16LE:
		return int64(binary.LittleEndian.Uint16(b)), 0, false
	case TypeU16BE:
		return int64(binary.BigEndian.Uint16(b)), 0, false
	case TypeI16LE:
		return int64(int16(binary.LittleEndian.Uint16(b))), 0, false
	case TypeI16BE:
		return int64(int16(binary.BigEndian.Uint16(b))), 0, false
	case TypeI32LE:
		return int64(int32(binary.LittleEndian.Uint32(b))), 0, false
	case TypeI32BE:
		return int64(int32(binary.BigEndian.Uint32(b))), 0, false
	case TypeU32LE:
		return int64(binary.LittleEndian.Uint32(b)), 0, false
	case TypeU32BE:
		return int64(binary.BigEndian.Uint32(b)), 0, false
	case TypeF32LE:
		return 0, float64(math.Float32frombits(binary.LittleEndian.Uint32(b))), true
	case TypeF32BE:
		return 0, float64(math.Float32frombits(binary.BigEndian.Uint32(b))), true
	}
	return 0, 0, false
}

// TimeOffset はフィールド i のサンプル時刻ずれを返す。
// SPI 転送時刻にこれを足すと、そのセンサが実際に値を取った推定時刻になる。
func (d *Decoder) TimeOffset(i int) time.Duration {
	return time.Duration(d.profile.Fields[i].TimeOffsetMs * float64(time.Millisecond))
}
