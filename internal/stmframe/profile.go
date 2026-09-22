// Package stmframe は STM から届く固定長フレームを、JSON のプロファイル定義に
// 従ってデコードする。
//
// バイト配置をコードに焼かないのは、計画 §12-B3 の通り IMU のバイト配置と
// スケールが未定だからである。決まった時点でプロファイルを書き換えるだけで
// 追従でき、過去に記録した生フレームも再デコードできる。
//
// 設計原則 (計画 §7.5): 定義されていないフィールドはセンサ非搭載として扱い、
// 上位はその観測を単に使わない。これにより IMU の仕様が来る前に、
// 車輪 + vision だけで推定を完成させて実機投入できる。
//
// このパッケージにビルドタグは付けない。開発 PC でそのままテストできる。
package stmframe

import (
	"encoding/json"
	"fmt"
	"io"
	"math"
	"os"
	"strconv"
	"strings"
)

// FieldType はフレーム上のバイト表現。
type FieldType string

// サポートする型。
const (
	TypeU8    FieldType = "u8"
	TypeI8    FieldType = "i8"
	TypeU16LE FieldType = "u16le"
	TypeU16BE FieldType = "u16be"
	TypeI16LE FieldType = "i16le"
	TypeI16BE FieldType = "i16be"
	TypeI32LE FieldType = "i32le"
	TypeI32BE FieldType = "i32be"
	TypeU32LE FieldType = "u32le"
	TypeU32BE FieldType = "u32be"
	TypeF32LE FieldType = "f32le"
	TypeF32BE FieldType = "f32be"
)

// Size はその型が占めるバイト数を返す。未知の型は 0。
func (t FieldType) Size() int {
	switch t {
	case TypeU8, TypeI8:
		return 1
	case TypeU16LE, TypeU16BE, TypeI16LE, TypeI16BE:
		return 2
	case TypeI32LE, TypeI32BE, TypeU32LE, TypeU32BE, TypeF32LE, TypeF32BE:
		return 4
	}
	return 0
}

// Byte は JSON 上で 255 とも "0xFF" とも書けるバイト値。
type Byte uint8

// UnmarshalJSON は数値と 16 進文字列の両方を受ける。
func (b *Byte) UnmarshalJSON(data []byte) error {
	s := strings.TrimSpace(string(data))
	if len(s) >= 2 && s[0] == '"' {
		var str string
		if err := json.Unmarshal(data, &str); err != nil {
			return err
		}
		v, err := strconv.ParseUint(strings.TrimPrefix(strings.TrimPrefix(str, "0x"), "0X"), 16, 8)
		if err != nil {
			return fmt.Errorf("parse byte %q: %w", str, err)
		}
		*b = Byte(v)
		return nil
	}
	var v uint8
	if err := json.Unmarshal(data, &v); err != nil {
		return err
	}
	*b = Byte(v)
	return nil
}

// MarshalJSON は "0xFF" 形式で書き出す。
func (b Byte) MarshalJSON() ([]byte, error) {
	return []byte(fmt.Sprintf("%q", fmt.Sprintf("0x%02X", uint8(b)))), nil
}

// Field は 1 つのセンサ値のフレーム上の配置と物理量への変換。
type Field struct {
	// Name は上位が参照する名前。プロファイル内で一意。
	Name string `json:"name"`
	// Offset はフレーム先頭からのバイト位置。
	Offset int `json:"offset"`
	// Type はバイト表現。
	Type FieldType `json:"type"`
	// Scale は生値に掛ける係数。value = raw*Scale + Bias。
	Scale float64 `json:"scale"`
	// Bias は変換後に足すオフセット。既定 0。
	Bias float64 `json:"bias"`
	// Unit は物理単位。ログのフィールド名に埋めるために持つ。
	// 単位を名前に書かなかったせいで実際に事故が起きている (計画 §3.9)。
	Unit string `json:"unit"`
	// TimeOffsetMs は「SPI 転送時刻に対して、この値が実際にサンプルされた時刻」の
	// ずれ [ms]。過去なので通常は負。STM のサンプリング周期の半分 + センサの
	// 群遅延 + 窓平均の半分 (計画 §5.2)。
	TimeOffsetMs float64 `json:"timeOffsetMs"`
}

// Reserved は「使っていないので特定の値でなければならない」領域。
//
// 現行フレームではパディングが 0 であることが、ヘッダ 0xFF / フッタ 0xAA だけでは
// 足りないフレーム同期の判別力を実質的に担っている。IMU をここに載せると
// その判別力が消えるので、代わりに CRC を入れてもらう必要がある (計画 §12-B3)。
type Reserved struct {
	Offset     int  `json:"offset"`
	Length     int  `json:"length"`
	MustBeZero bool `json:"mustBeZero"`
}

// Profile はフレーム 1 種類の定義。
type Profile struct {
	// Name はプロファイルの識別子。ログに残して後から再デコードするために使う。
	Name string `json:"profile"`
	// FrameSize はフレーム全体のバイト数。
	FrameSize int `json:"frameSize"`
	// Header / Footer は先頭・末尾のマーカー。省略すると検査しない。
	Header *Byte `json:"header,omitempty"`
	Footer *Byte `json:"footer,omitempty"`

	Fields   []Field    `json:"fields"`
	Reserved []Reserved `json:"reserved,omitempty"`
}

// LoadProfile は JSON からプロファイルを読み、妥当性を検査する。
func LoadProfile(r io.Reader) (*Profile, error) {
	var p Profile
	dec := json.NewDecoder(r)
	dec.DisallowUnknownFields()
	if err := dec.Decode(&p); err != nil {
		return nil, fmt.Errorf("decode profile: %w", err)
	}
	if err := p.Validate(); err != nil {
		return nil, err
	}
	return &p, nil
}

// LoadProfileFile は path の JSON からプロファイルを読む。
func LoadProfileFile(path string) (*Profile, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, fmt.Errorf("open profile %s: %w", path, err)
	}
	defer f.Close()
	p, err := LoadProfile(f)
	if err != nil {
		return nil, fmt.Errorf("%s: %w", path, err)
	}
	return p, nil
}

// Validate はプロファイルが自己矛盾していないかを調べる。
func (p *Profile) Validate() error {
	if p.Name == "" {
		return fmt.Errorf("profile name is empty")
	}
	if p.FrameSize <= 0 {
		return fmt.Errorf("frameSize must be positive, got %d", p.FrameSize)
	}
	if len(p.Fields) == 0 {
		return fmt.Errorf("profile %s has no fields", p.Name)
	}

	seen := make(map[string]struct{}, len(p.Fields))
	for i := range p.Fields {
		f := &p.Fields[i]
		if f.Name == "" {
			return fmt.Errorf("field[%d] has no name", i)
		}
		if _, dup := seen[f.Name]; dup {
			return fmt.Errorf("duplicate field name %q", f.Name)
		}
		seen[f.Name] = struct{}{}

		size := f.Type.Size()
		if size == 0 {
			return fmt.Errorf("field %q has unknown type %q", f.Name, f.Type)
		}
		if f.Offset < 0 || f.Offset+size > p.FrameSize {
			return fmt.Errorf("field %q at offset %d (%d bytes) does not fit in a %d-byte frame",
				f.Name, f.Offset, size, p.FrameSize)
		}
		if f.Scale == 0 || math.IsNaN(f.Scale) {
			return fmt.Errorf("field %q has scale %v; a zero scale would silently discard the sensor", f.Name, f.Scale)
		}
	}

	for i, r := range p.Reserved {
		if r.Length <= 0 {
			return fmt.Errorf("reserved[%d] has non-positive length %d", i, r.Length)
		}
		if r.Offset < 0 || r.Offset+r.Length > p.FrameSize {
			return fmt.Errorf("reserved[%d] at offset %d (%d bytes) does not fit in a %d-byte frame",
				i, r.Offset, r.Length, p.FrameSize)
		}
	}
	return nil
}

// SyncStrength はフレーム同期にどれだけの判別力があるかを表す、
// 「一致してほしいバイト数」の概算。
//
// ヘッダとフッタだけ (2 バイト) だと 20 バイトのランダムなずれに対して
// 偽陽性が 1/65536 で起きる。8 ms 周期で回していると数時間に 1 回は引く。
// 現行プロファイルはパディング 7 バイトがこれを 9 バイトへ押し上げている。
func (p *Profile) SyncStrength() int {
	n := 0
	if p.Header != nil {
		n++
	}
	if p.Footer != nil {
		n++
	}
	for _, r := range p.Reserved {
		if r.MustBeZero {
			n += r.Length
		}
	}
	return n
}
