package stmframe

import (
	"bytes"
	"embed"
	"fmt"
	"path"
	"sort"
	"strings"
)

//go:embed profiles/*.json
var builtinFS embed.FS

const builtinDir = "profiles"

// DefaultProfileName は実機で現在動いているフレーム (IMU なし) のプロファイル。
//
// 計画 §3.5 の通り STM 側に IMU の実装が存在しないため、これが既定である。
// IMU が載ったら rock5a-v2-imu へ切り替える。
const DefaultProfileName = "rock5a-v1"

// Builtin は埋め込みのプロファイルを名前で読む。
//
// 実機へ JSON を配らなくても動くようにバイナリへ埋め込んである。
// 現場でバイト配置を試すときは LoadProfileFile で外部ファイルを使う。
func Builtin(name string) (*Profile, error) {
	data, err := builtinFS.ReadFile(path.Join(builtinDir, name+".json"))
	if err != nil {
		return nil, fmt.Errorf("stmframe: no builtin profile %q (have %v)", name, BuiltinNames())
	}
	p, err := LoadProfile(bytes.NewReader(data))
	if err != nil {
		return nil, fmt.Errorf("builtin profile %s: %w", name, err)
	}
	return p, nil
}

// BuiltinNames は埋め込まれているプロファイル名を返す。
func BuiltinNames() []string {
	entries, err := builtinFS.ReadDir(builtinDir)
	if err != nil {
		return nil
	}
	names := make([]string, 0, len(entries))
	for _, e := range entries {
		names = append(names, strings.TrimSuffix(e.Name(), ".json"))
	}
	sort.Strings(names)
	return names
}

// Resolve はプロファイルを解決する。spec が空なら DefaultProfileName、
// ".json" で終わればファイルパス、それ以外は埋め込みの名前として扱う。
func Resolve(spec string) (*Profile, error) {
	switch {
	case spec == "":
		return Builtin(DefaultProfileName)
	case strings.HasSuffix(spec, ".json"):
		return LoadProfileFile(spec)
	default:
		return Builtin(spec)
	}
}
