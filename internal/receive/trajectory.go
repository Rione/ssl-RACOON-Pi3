package receive

import (
	"fmt"

	"github.com/Rione/ssl-RACOON-Pi3/internal/control"
	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// Plan はデコード・時計変換後の不変な経路。wireの形式やcommand_idではない。
// パケット長、送信者、epoch、seq検証は将来のUDPデコーダで実装する。
// 有効期限はパケット再送のたびに延長してはならない。
type Plan struct {
	id         uint64
	validUntil localization.Stamp
	controller *control.Controller
}

// PreparePlan は受信更新時にだけ経路を検証・コピーする。
// 呼び出し側が安全に公開し、制御周期ではControllerを再構築しない。
func PreparePlan(id uint64, validUntil localization.Stamp, nodes []control.Node, cfg control.Config) (*Plan, error) {
	if len(nodes) < 2 || validUntil <= nodes[0].Stamp {
		return nil, fmt.Errorf("invalid trajectory lifetime")
	}
	c, err := control.New(nodes, cfg)
	if err != nil {
		return nil, err
	}
	return &Plan{id: id, validUntil: validUntil, controller: c}, nil
}

func (p *Plan) ID() uint64 {
	if p == nil {
		return 0
	}
	return p.id
}
func (p *Plan) ValidUntil() localization.Stamp {
	if p == nil {
		return 0
	}
	return p.validUntil
}
func (p *Plan) Controller() *control.Controller {
	if p == nil {
		return nil
	}
	return p.controller
}
