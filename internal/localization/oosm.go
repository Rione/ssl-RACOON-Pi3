package localization

// 遅延観測の扱い (計画 §4.6)。
//
// vision は片道遅延ぶん遅れて届く。現在時刻にそのまま当てると、
// 2 m/s のロボットで 20 ms の遅延なら 40 mm の系統誤差になる。
// 最適化ではなく正しさの問題なので、最小構成にも入れてある。
//
// **計画からの意図的な逸脱**: 計画 §4.6 は「現在状態を t_v へ retrodict し、
// 補正を現在時刻へ伝播する。全区間の再伝播はしない」としている。ここでは
// **バッファ区間の再フィルタ**を行う。理由は次の通り。
//
//   - 計画が全区間再処理を避ける理由は計算コストだが、200 ms = 25 ステップの
//     再処理は 8x8 の行列演算 25 回で、60 Hz で回しても 10 Mflops/s 程度。
//     Rock5A では問題にならない。
//   - 計画自身が引く文献では、retrodiction は全区間再処理に対して共分散が
//     2〜3% 悪化する**近似**である。安く済むなら良いほうを採るのが筋。
//   - 後退遷移行列を作る必要がなく、実装が単純で誤りが入りにくい。

// bufferEntry は 1 周期ぶんの状態とその周期に使った観測。
//
// 再フィルタのために「その時刻の事後状態」と「そこへ至るのに使った入力」を
// 両方持つ。
type bufferEntry struct {
	stamp Stamp
	x     nominal
	p     [stateDim][stateDim]float64

	// dt は 1 つ前のエントリからの経過。
	dt float64
	// wheels はこの周期で取り込んだ車輪観測 (論理輪番号の順)。
	wheels    [NumWheels]float64
	hasWheels bool
	// gyro はこの周期で取り込んだジャイロのヨーレート [rad/s]。
	gyro    float64
	hasGyro bool
}

// ringBuffer は固定長の循環バッファ。起動時に確保して使い回す。
type ringBuffer struct {
	entries []bufferEntry
	start   int
	count   int
}

func newRingBuffer(n int) ringBuffer {
	if n < 1 {
		n = 1
	}
	return ringBuffer{entries: make([]bufferEntry, n)}
}

func (r *ringBuffer) len() int { return r.count }

// at は古い順に i 番目のエントリを返す。
func (r *ringBuffer) at(i int) *bufferEntry {
	return &r.entries[(r.start+i)%len(r.entries)]
}

// push は末尾に追加する。満杯なら最も古いものを捨てる。
func (r *ringBuffer) push(e bufferEntry) {
	if r.count < len(r.entries) {
		*r.at(r.count) = e
		r.count++
		return
	}
	*r.at(0) = e
	r.start = (r.start + 1) % len(r.entries)
}

func (r *ringBuffer) reset() {
	r.start = 0
	r.count = 0
}

// oldest はバッファに残っている最も古い時刻を返す。
func (r *ringBuffer) oldest() (Stamp, bool) {
	if r.count == 0 {
		return 0, false
	}
	return r.at(0).stamp, true
}

// findAtOrBefore は stamp 以下で最も新しいエントリの位置を返す。
// 無ければ -1。
func (r *ringBuffer) findAtOrBefore(stamp Stamp) int {
	lo, hi := 0, r.count-1
	found := -1
	for lo <= hi {
		mid := (lo + hi) / 2
		if r.at(mid).stamp <= stamp {
			found = mid
			lo = mid + 1
		} else {
			hi = mid - 1
		}
	}
	return found
}
