package loclog

import (
	"encoding/json"
	"fmt"
	"io"
	"os"

	"github.com/foxglove/mcap/go/mcap"
)

// Reader は記録した MCAP を読み返す。
//
// 同じ MCAP を流せば同じ推定結果が出ることが、オフラインチューニングと
// 回帰テストの前提になる (計画 §7.4 / §9)。
type Reader struct {
	file *os.File
	r    *mcap.Reader
}

// OpenReader は MCAP を開く。
func OpenReader(path string) (*Reader, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, fmt.Errorf("loclog: open %s: %w", path, err)
	}
	r, err := mcap.NewReader(f)
	if err != nil {
		f.Close()
		return nil, fmt.Errorf("loclog: read %s: %w", path, err)
	}
	return &Reader{file: f, r: r}, nil
}

// Close はファイルを閉じる。
func (r *Reader) Close() error {
	r.r.Close()
	return r.file.Close()
}

// Metadata は記録時に書いたアンカーなどを返す。
func (r *Reader) Metadata() (map[string]string, error) {
	info, err := r.r.Info()
	if err != nil {
		return nil, fmt.Errorf("loclog: info: %w", err)
	}
	out := map[string]string{}
	for _, idx := range info.MetadataIndexes {
		md, err := r.r.GetMetadata(idx.Offset)
		if err != nil {
			return nil, fmt.Errorf("loclog: metadata: %w", err)
		}
		for k, v := range md.Metadata {
			out[k] = v
		}
	}
	return out, nil
}

// ChannelCounts はトピックごとのメッセージ数を返す。
func (r *Reader) ChannelCounts() (map[string]uint64, error) {
	info, err := r.r.Info()
	if err != nil {
		return nil, fmt.Errorf("loclog: info: %w", err)
	}
	return info.ChannelCounts(), nil
}

// Each は topics に含まれるメッセージを時刻順に渡す。topics が空なら全部。
//
// data は次の呼び出しで上書きされ得るので、保持するならコピーすること。
func (r *Reader) Each(topics []string, fn func(topic string, logTimeNs uint64, data []byte) error) error {
	var opts []mcap.ReadOpt
	if len(topics) > 0 {
		opts = append(opts, mcap.WithTopics(topics))
	}
	it, err := r.r.Messages(opts...)
	if err != nil {
		return fmt.Errorf("loclog: iterate: %w", err)
	}
	msg := &mcap.Message{}
	for {
		_, ch, m, err := it.NextInto(msg)
		if err == io.EOF {
			return nil
		}
		if err != nil {
			return fmt.Errorf("loclog: read message: %w", err)
		}
		if err := fn(ch.Topic, m.LogTime, m.Data); err != nil {
			return err
		}
	}
}

// ReadJSONInto は 1 トピックの JSON メッセージをすべて読み、T のスライスにする。
func ReadJSONInto[T any](r *Reader, topic string) ([]T, error) {
	var out []T
	err := r.Each([]string{topic}, func(_ string, _ uint64, data []byte) error {
		var v T
		if err := json.Unmarshal(data, &v); err != nil {
			return fmt.Errorf("loclog: decode %s: %w", topic, err)
		}
		out = append(out, v)
		return nil
	})
	if err != nil {
		return nil, err
	}
	return out, nil
}
