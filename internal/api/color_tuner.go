package api

import (
	"encoding/json"
	"fmt"
	"io"
	"log"
	"net"
	"strconv"
	"strings"
	"time"

	"github.com/Rione/ssl-RACOON-Pi2/internal/mw"
	"github.com/Rione/ssl-RACOON-Pi2/internal/state"
)

func handleColorTunerPage(conn net.Conn) {
	sendHTTPResponse(conn, 200, "text/html", colorTunerHTML)
}

func handleColorPreview(conn net.Conn) {
	body, err := requestTuner("preview")
	if err != nil {
		sendHTTPResponse(conn, 503, "application/json", fmt.Sprintf(
			`{"ok":false,"error":%q}`, err.Error(),
		))
		return
	}
	sendHTTPResponse(conn, 200, "application/json", string(body))
}

func handleColorThresholds(conn net.Conn) {
	adj := mw.GetAdjustment()
	body, err := json.Marshal(adj)
	if err != nil {
		sendErrorResponse(conn, 500)
		return
	}
	sendHTTPResponse(conn, 200, "application/json", string(body))
}

func handleSetColor(conn net.Conn, pathParts []string) {
	if len(pathParts) < 7 {
		sendErrorResponse(conn, 400)
		return
	}

	minThreshold := pathParts[2]
	maxThreshold := pathParts[3]
	ballDetectRadius, err := strconv.Atoi(pathParts[4])
	if err != nil {
		sendErrorResponse(conn, 400)
		return
	}
	circularityThreshold, err := strconv.ParseFloat(pathParts[5], 32)
	if err != nil {
		sendErrorResponse(conn, 400)
		return
	}
	save := pathParts[6] == "1"

	cmd := fmt.Sprintf(
		"set|%s|%s|%d|%.4f|%s",
		minThreshold,
		maxThreshold,
		ballDetectRadius,
		circularityThreshold,
		boolTo01(save),
	)

	body, err := requestTuner(cmd)
	if err != nil {
		log.Printf("setcolor error: %v", err)
		sendErrorResponse(conn, 500)
		return
	}

	if save {
		mw.ReloadAdjustment()
	}

	sendHTTPResponse(conn, 200, "application/json", string(body))
}

func handleRelaxColor(conn net.Conn, pathParts []string) {
	if len(pathParts) < 3 {
		sendErrorResponse(conn, 400)
		return
	}
	save := pathParts[2] == "1"

	body, err := requestTuner("relax|" + boolTo01(save))
	if err != nil {
		log.Printf("relaxcolor error: %v", err)
		sendErrorResponse(conn, 500)
		return
	}

	if save {
		mw.ReloadAdjustment()
	}

	sendHTTPResponse(conn, 200, "application/json", string(body))
}

func boolTo01(v bool) string {
	if v {
		return "1"
	}
	return "0"
}

func requestTuner(command string) ([]byte, error) {
	addr := fmt.Sprintf("127.0.0.1:%d", state.TunerPort)
	timeout := 10 * time.Second
	if command == "preview" {
		timeout = 20 * time.Second
	}

	var lastErr error
	for attempt := 0; attempt < 5; attempt++ {
		if attempt > 0 {
			time.Sleep(200 * time.Millisecond)
		}

		body, err := requestTunerOnce(addr, command, timeout)
		if err == nil {
			return body, nil
		}
		lastErr = err
	}
	return nil, lastErr
}

func requestTunerOnce(addr, command string, timeout time.Duration) ([]byte, error) {
	conn, err := net.DialTimeout("tcp", addr, 3*time.Second)
	if err != nil {
		return nil, fmt.Errorf("カメラチューナーへ接続できませんでした: %w", err)
	}
	defer conn.Close()

	if err := conn.SetDeadline(time.Now().Add(timeout)); err != nil {
		return nil, err
	}

	if _, err := conn.Write([]byte(command + "\n")); err != nil {
		return nil, fmt.Errorf("チューナー要求の送信に失敗しました: %w", err)
	}

	var buf strings.Builder
	tmp := make([]byte, 65536)
	for {
		n, err := conn.Read(tmp)
		if n > 0 {
			buf.Write(tmp[:n])
			if strings.Contains(buf.String(), "\n") {
				break
			}
		}
		if err != nil {
			if err == io.EOF && buf.Len() > 0 {
				break
			}
			return nil, fmt.Errorf("チューナー応答の受信に失敗しました: %w", err)
		}
	}

	trimmed := strings.TrimSpace(buf.String())
	if trimmed == "" {
		return nil, fmt.Errorf("チューナー応答が空です")
	}

	return []byte(trimmed), nil
}

const colorTunerHTML = `<!DOCTYPE html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>HSV Color Tuner</title>
  <style>
    * { box-sizing: border-box; }
    body { font-family: sans-serif; margin: 0; padding: 16px; background: #111; color: #eee; }
    h1 { font-size: 1.2rem; margin: 0 0 12px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    @media (max-width: 900px) { .grid { grid-template-columns: 1fr; } }
    img { width: 100%; background: #222; border-radius: 8px; min-height: 180px; object-fit: contain; }
    .panel { background: #1a1a1a; border-radius: 8px; padding: 12px; margin-bottom: 12px; }
    label { display: block; margin: 8px 0 4px; font-size: 0.85rem; color: #bbb; }
    input[type=range] { width: 100%; }
    .row { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }
    .val { font-family: monospace; color: #9f9; }
    .btns { display: flex; flex-wrap: wrap; gap: 8px; margin-top: 12px; }
    button { padding: 8px 14px; border: 0; border-radius: 6px; cursor: pointer; background: #2563eb; color: #fff; }
    button.secondary { background: #374151; }
    button.warn { background: #b45309; }
    #status { font-size: 0.85rem; color: #aaa; min-height: 1.2em; margin-top: 8px; }
  </style>
</head>
<body>
  <h1>HSV 色検出チューナー</h1>
  <div id="status">loading…</div>

  <div class="grid">
    <div class="panel">
      <div>カメラ + 検出</div>
      <img id="cam" alt="camera preview">
    </div>
    <div class="panel">
      <div>マスク（緑 = 検出色）</div>
      <img id="mask" alt="mask preview">
    </div>
  </div>

  <div class="panel">
    <div>Min HSV <span class="val" id="minLabel"></span></div>
    <div class="row">
      <div><label>H <span id="minHv"></span></label><input id="minH" type="range" min="0" max="179" value="0"></div>
      <div><label>S <span id="minSv"></span></label><input id="minS" type="range" min="0" max="255" value="0"></div>
      <div><label>V <span id="minVv"></span></label><input id="minV" type="range" min="0" max="255" value="0"></div>
    </div>
    <div>Max HSV <span class="val" id="maxLabel"></span></div>
    <div class="row">
      <div><label>H <span id="maxHv"></span></label><input id="maxH" type="range" min="0" max="179" value="179"></div>
      <div><label>S <span id="maxSv"></span></label><input id="maxS" type="range" min="0" max="255" value="255"></div>
      <div><label>V <span id="maxVv"></span></label><input id="maxV" type="range" min="0" max="255" value="255"></div>
    </div>
    <label>ballDetectRadius <span id="radiusV"></span></label>
    <input id="radius" type="range" min="50" max="300" value="150">
    <label>circularityThreshold <span id="circV"></span></label>
    <input id="circ" type="range" min="0.05" max="0.8" step="0.01" value="0.2">
    <div class="btns">
      <button id="applyBtn">適用（プレビューのみ）</button>
      <button id="saveBtn">適用して保存</button>
      <button class="warn" id="relaxBtn">少し緩める</button>
      <button class="secondary" id="reloadBtn">ファイルから再読込</button>
    </div>
  </div>

<script>
const ids = ["minH","minS","minV","maxH","maxS","maxV","radius","circ"];
const statusEl = document.getElementById("status");

function parseTriple(s) {
  const p = s.split(",").map(x => parseInt(x.trim(), 10));
  return [p[0]||0, p[1]||0, p[2]||0];
}
function triple(arr) { return arr.join(","); }

function readSliders() {
  return {
    min: [+minH.value, +minS.value, +minV.value],
    max: [+maxH.value, +maxS.value, +maxV.value],
    radius: +radius.value,
    circ: +circ.value,
  };
}

function writeSliders(minS, maxS, radiusV, circV) {
  const [mh,ms,mv] = parseTriple(minS);
  const [xh,xs,xv] = parseTriple(maxS);
  minH.value = mh; minS.value = ms; minV.value = mv;
  maxH.value = xh; maxS.value = xs; maxV.value = xv;
  radius.value = radiusV;
  circ.value = circV;
  updateLabels();
}

function updateLabels() {
  const s = readSliders();
  minHv.textContent = s.min[0]; minSv.textContent = s.min[1]; minVv.textContent = s.min[2];
  maxHv.textContent = s.max[0]; maxSv.textContent = s.max[1]; maxVv.textContent = s.max[2];
  minLabel.textContent = triple(s.min);
  maxLabel.textContent = triple(s.max);
  radiusV.textContent = s.radius;
  circV.textContent = s.circ.toFixed(2);
}

ids.forEach(id => document.getElementById(id).addEventListener("input", updateLabels));

async function loadThresholds() {
  const r = await fetch("/colorthresholds");
  const j = await r.json();
  writeSliders(j.minThreshold, j.maxThreshold, j.ballDetectRadius, j.circularityThreshold);
}

async function refreshPreview() {
  try {
    const r = await fetch("/colorpreview");
    const j = await r.json();
    if (!j.ok) { statusEl.textContent = j.error || "preview failed"; return; }
    if (j.cameraFrame) cam.src = "data:image/jpeg;base64," + j.cameraFrame;
    if (j.maskFrame) mask.src = "data:image/jpeg;base64," + j.maskFrame;
    statusEl.textContent = j.isball
      ? ("ball detected  x=" + j.x.toFixed(1) + " y=" + j.y.toFixed(1))
      : "ball not detected";
  } catch (e) {
    statusEl.textContent = "preview error: " + e;
  }
}

async function apply(save) {
  const s = readSliders();
  const url = "/setcolor/" + triple(s.min) + "/" + triple(s.max) + "/" + s.radius + "/" + s.circ + "/" + (save ? "1" : "0");
  const r = await fetch(url);
  const j = await r.json();
  if (!j.ok) { statusEl.textContent = j.error || "apply failed"; return; }
  statusEl.textContent = save ? "saved" : "applied";
  await refreshPreview();
}

async function relax() {
  const r = await fetch("/relaxcolor/0");
  const j = await r.json();
  if (!j.ok) { statusEl.textContent = j.error || "relax failed"; return; }
  writeSliders(j.minThreshold, j.maxThreshold, j.ballDetectRadius, j.circularityThreshold);
  statusEl.textContent = "relaxed (preview)";
  await refreshPreview();
}

applyBtn.onclick = () => apply(false);
saveBtn.onclick = () => apply(true);
relaxBtn.onclick = () => relax();
reloadBtn.onclick = async () => { await loadThresholds(); await refreshPreview(); };

loadThresholds().then(refreshPreview);
setInterval(refreshPreview, 400);
</script>
</body>
</html>`
