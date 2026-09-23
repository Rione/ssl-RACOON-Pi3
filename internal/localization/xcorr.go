package localization

import "math"

// 2 つの時系列のあいだの時間ずれを相互相関で測る道具。
//
// 使い道が 2 つある (docs/self-localization-research-20260923.md §3.5, §4.4):
//
//   - **vision の定数遅延**: 車輪から出した速度と vision から出した速度のずれ。
//     片方向の観測だけでは「クロックオフセット」と「最小片方向遅延」を分離
//     できないが、**車輪という独立した速度源があれば相関で決まる**
//     (Mair の相互相関法、Kelly & Sukhatme の回転曲線の位置合わせ、
//     Qin & Shen のオンライン推定と同じ原理)。
//     PoC が実際にこの方法で「車輪を 8-12 ms 古いとすると最も合う」と測っている。
//   - **指令が効くまでの遅れ**: 出した速度指令と実測の速度のずれ。
//
// どちらも「推定器の状態には入れない」。時定数が姿勢推定と 4-5 桁違うので、
// 同じ状態ベクトルに混ぜると数値的に条件が悪くなる (計画 §5.3 と同じ理屈)。

// lagResult は相互相関の結果。
type lagResult struct {
	// LagSamples はサンプル単位の遅れ (放物線補間ずみ)。
	// 正なら b が a より遅れている。
	LagSamples float64
	// Peak は正規化相関のピーク値 [-1, 1]。
	Peak float64
	// OK はピークが有効かどうか。
	OK bool
}

// crossCorrelateLag は a を基準に b がどれだけ遅れているかを返す。
//
// a, b は同じ時間格子に並んだ同じ長さの列。valid が false の要素は無視する。
// lag は minLag..maxLag の範囲で探索する (負も可)。
//
// 平均を引いてから相関を取るので、定数オフセットには影響されない。
// **スケールの違いにも影響されない** (正規化相関なので)。これは
// 「車輪と vision で速度の単位系が少しずれていても遅れは測れる」ことを意味する。
func crossCorrelateLag(a, b []float64, validA, validB []bool, minLag, maxLag int) lagResult {
	n := len(a)
	if n == 0 || len(b) != n || len(validA) != n || len(validB) != n {
		return lagResult{}
	}
	if minLag > maxLag {
		return lagResult{}
	}

	corr := make([]float64, maxLag-minLag+1)
	best, bestIdx := -2.0, -1
	for lag := minLag; lag <= maxLag; lag++ {
		// 重なる区間だけで平均・分散・相関を計算する。
		var sa, sb float64
		var count int
		for i := 0; i < n; i++ {
			j := i + lag
			if j < 0 || j >= n || !validA[i] || !validB[j] {
				continue
			}
			sa += a[i]
			sb += b[j]
			count++
		}
		if count < 16 {
			corr[lag-minLag] = -2
			continue
		}
		ma := sa / float64(count)
		mb := sb / float64(count)
		var num, va, vb float64
		for i := 0; i < n; i++ {
			j := i + lag
			if j < 0 || j >= n || !validA[i] || !validB[j] {
				continue
			}
			da := a[i] - ma
			db := b[j] - mb
			num += da * db
			va += da * da
			vb += db * db
		}
		if va <= 0 || vb <= 0 {
			corr[lag-minLag] = -2
			continue
		}
		c := num / (math.Sqrt(va) * math.Sqrt(vb))
		corr[lag-minLag] = c
		if c > best {
			best, bestIdx = c, lag-minLag
		}
	}
	if bestIdx < 0 {
		return lagResult{}
	}

	lag := float64(minLag + bestIdx)
	// 放物線当てはめでサンプル間を補間する。相関の山は滑らかなので、
	// これで 1 サンプルよりずっと細かい分解能が出る。
	if bestIdx > 0 && bestIdx < len(corr)-1 {
		y0, y1, y2 := corr[bestIdx-1], corr[bestIdx], corr[bestIdx+1]
		if y0 > -2 && y2 > -2 {
			if d := y0 - 2*y1 + y2; d != 0 {
				if adj := 0.5 * (y0 - y2) / d; adj > -1 && adj < 1 {
					lag += adj
				}
			}
		}
	}
	return lagResult{LagSamples: lag, Peak: best, OK: true}
}
