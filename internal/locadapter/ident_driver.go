package locadapter

import (
	"log"
	"sync/atomic"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/localization"
	"github.com/Rione/ssl-RACOON-Pi2/internal/loclog"
)

// IdentDriver は加振シーケンスをリンク層の速度指令へ流し込む。
//
// これはロボットを実際に走らせる。明示的なフラグでしか起動しないこと。
// 非常停止が立っているフレームには link 側が触らせない。
type IdentDriver struct {
	ex    *Exciter
	clock *loclog.Clock
	rec   *loclog.Writer

	start   localization.Stamp
	started atomic.Bool
	done    atomic.Bool

	lastSegment atomic.Int64
}

// NewIdentDriver は加振ドライバを作る。
func NewIdentDriver(clock *loclog.Clock, rec *loclog.Writer, cfg ExciteConfig) *IdentDriver {
	d := &IdentDriver{ex: NewExciter(cfg), clock: clock, rec: rec}
	d.lastSegment.Store(-1)
	return d
}

// Total はシーケンス全体の長さを返す。
func (d *IdentDriver) Total() time.Duration { return d.ex.Total() }

// Done はシーケンスが終わったかを返す。
func (d *IdentDriver) Done() bool { return d.done.Load() }

// OverrideVelocity は link.VelocityOverride を満たす。
func (d *IdentDriver) OverrideVelocity() (velX, velY, velAng int16, ok bool) {
	now := d.clock.Now()
	if d.started.CompareAndSwap(false, true) {
		d.start = now
		log.Printf("[IDENT] excitation started: %d segments, %v total",
			d.ex.Segments(), d.ex.Total())
	}

	c := d.ex.At(now.Sub(d.start))
	if c.Done {
		if d.done.CompareAndSwap(false, true) {
			log.Printf("[IDENT] excitation finished; run cmd/loc_ident on the recorded MCAP")
		}
		// 終わっても 0 を出し続ける。指令を返さなくなると通常経路へ戻り、
		// 直前の速度が残っていた場合に走り去る。
		return 0, 0, 0, true
	}

	if prev := d.lastSegment.Swap(int64(c.Segment)); prev != int64(c.Segment) {
		log.Printf("[IDENT] segment %d/%d: %s (vx %.2f vy %.2f omega %.2f)",
			c.Segment+1, d.ex.Segments(), c.Label, c.VX, c.VY, c.Omega)
		if d.rec != nil {
			d.rec.LogJSON(loclog.ChControlTarget, now, struct {
				Source    string  `json:"source"`
				Segment   int     `json:"segment"`
				Label     string  `json:"label"`
				VXMS      float64 `json:"vx_m_s"`
				VYMS      float64 `json:"vy_m_s"`
				OmegaRadS float64 `json:"omega_rad_s"`
				Resting   bool    `json:"resting"`
			}{"ident", c.Segment, c.Label, c.VX, c.VY, c.Omega, c.Resting})
		}
	}

	velX, velY, velAng = c.ToFrameUnits()
	return velX, velY, velAng, true
}
