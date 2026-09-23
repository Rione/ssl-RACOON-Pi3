package control

import (
	"fmt"
	"sort"

	"github.com/Rione/ssl-RACOON-Pi3/internal/localization"
)

// Node は未来の経路点。Stamp は送信時刻ではなく到達予定時刻であり、
// 呼び出し前に Estimate.Stamp と同じロボット側の単調時間軸へ変換する。
type Node struct {
	Stamp   localization.Stamp
	Pose    localization.Pose2 // ワールド系 [m, m, rad]
	VelBody localization.Vec2  // このノードの姿勢を基準とする目標速度 [m/s]
	YawRate float64            // 目標角速度 [rad/s]
}

// Reference は同じ時刻の経路参照。速度はworld系。通信形式から独立する。
type Reference struct {
	Stamp    localization.Stamp
	Pose     localization.Pose2
	VelWorld localization.Vec2
	YawRate  float64
}

func validateNodes(nodes []Node) ([]Node, error) {
	copyNodes := append([]Node(nil), nodes...)
	for i, n := range copyNodes {
		if n.Stamp < 0 || !finite(n.Pose.X, n.Pose.Y, n.Pose.Theta, n.VelBody.X, n.VelBody.Y, n.YawRate) {
			return nil, fmt.Errorf("invalid trajectory node %d", i)
		}
		if i > 0 && n.Stamp <= copyNodes[i-1].Stamp {
			return nil, fmt.Errorf("node %d arrival time must strictly increase", i)
		}
		copyNodes[i].Pose.Theta = localization.WrapAngle(n.Pose.Theta)
	}
	return copyNodes, nil
}

// ReferenceAt は不変の経路を評価する。開始前・終端後にはゼロ参照と状態を返す。
// Newで構築していないControllerではWaitingとなる。
func (c *Controller) ReferenceAt(now localization.Stamp) (Reference, Phase) {
	if c == nil || len(c.nodes) < 2 {
		return Reference{}, Waiting
	}
	return sampleTrajectory(c.nodes, now)
}

func sampleTrajectory(nodes []Node, now localization.Stamp) (Reference, Phase) {
	if now < nodes[0].Stamp {
		return Reference{}, Waiting
	}
	if now >= nodes[len(nodes)-1].Stamp {
		return Reference{}, Finished
	}
	i := sort.Search(len(nodes), func(i int) bool { return nodes[i].Stamp > now })
	a, b := nodes[i-1], nodes[i]
	u := float64(now-a.Stamp) / float64(b.Stamp-a.Stamp)
	lerp := func(a, b float64) float64 { return (1-u)*a + u*b }
	x, y := lerp(a.Pose.X, b.Pose.X), lerp(a.Pose.Y, b.Pose.Y)
	theta := localization.WrapAngle(a.Pose.Theta + u*localization.AngleDiff(b.Pose.Theta, a.Pose.Theta))
	va := localization.Rotate(a.Pose.Theta, a.VelBody)
	vb := localization.Rotate(b.Pose.Theta, b.VelBody)
	return Reference{Stamp: now, Pose: localization.Pose2{X: x, Y: y, Theta: theta},
		VelWorld: localization.Vec2{X: lerp(va.X, vb.X), Y: lerp(va.Y, vb.Y)},
		YawRate:  lerp(a.YawRate, b.YawRate)}, Tracking
}
