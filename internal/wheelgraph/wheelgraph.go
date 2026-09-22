//go:build pi4 || rock5a

package wheelgraph

import (
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"sync"
	"time"
)

const (
	maxSamples = 600
	port       = ":9192"
)

// Sample is one Wheel(raw) reading from SPI/UART.
type Sample struct {
	T  int64 `json:"t"`
	FL int16 `json:"fl"`
	BL int16 `json:"bl"`
	BR int16 `json:"br"`
	FR int16 `json:"fr"`
}

var (
	mu       sync.RWMutex
	samples  []Sample
	enabled  bool
)

// SetEnabled toggles recording and the graph HTTP server target.
func SetEnabled(on bool) {
	mu.Lock()
	enabled = on
	if !on {
		samples = nil
	}
	mu.Unlock()
}

// Record stores a Wheel(raw) sample when graph mode is enabled.
func Record(fl, bl, br, fr int16) {
	if !enabled {
		return
	}

	mu.Lock()
	defer mu.Unlock()

	s := Sample{
		T:  time.Now().UnixMilli(),
		FL: fl,
		BL: bl,
		BR: br,
		FR: fr,
	}
	if len(samples) >= maxSamples {
		samples = append(samples[1:], s)
		return
	}
	samples = append(samples, s)
}

// RunServer serves a live Chart.js page and JSON samples until done is closed.
func RunServer(done <-chan struct{}) {
	mux := http.NewServeMux()
	mux.HandleFunc("/wheel-graph", handleGraphPage)
	mux.HandleFunc("/wheel-raw.json", handleSamplesJSON)

	srv := &http.Server{
		Addr:    port,
		Handler: mux,
	}

	go func() {
		<-done
		_ = srv.Close()
	}()

	log.Printf("Wheel(raw) graph: http://<robot>%s/wheel-graph (-dw)", port)
	if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
		log.Printf("Wheel graph server error: %v", err)
	}
}

func handleSamplesJSON(w http.ResponseWriter, _ *http.Request) {
	mu.RLock()
	out := append([]Sample(nil), samples...)
	mu.RUnlock()

	w.Header().Set("Content-Type", "application/json")
	w.Header().Set("Access-Control-Allow-Origin", "*")
	_ = json.NewEncoder(w).Encode(map[string]any{"samples": out})
}

func handleGraphPage(w http.ResponseWriter, _ *http.Request) {
	w.Header().Set("Content-Type", "text/html; charset=utf-8")
	fmt.Fprint(w, graphPageHTML)
}

const graphPageHTML = `<!DOCTYPE html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Wheel(raw) Graph</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/chart.umd.min.js"></script>
  <style>
    body { font-family: sans-serif; margin: 16px; background: #111; color: #eee; }
    h1 { font-size: 1.1rem; margin: 0 0 12px; }
    #meta { font-size: 0.85rem; color: #aaa; margin-bottom: 12px; }
    canvas { background: #1a1a1a; border-radius: 8px; max-height: 70vh; }
  </style>
</head>
<body>
  <h1>Wheel(raw) — FL / BL / BR / FR</h1>
  <div id="meta">loading…</div>
  <canvas id="chart"></canvas>
  <script>
    const colors = { fl: "#4ade80", bl: "#60a5fa", br: "#f472b6", fr: "#fbbf24" };
    const ctx = document.getElementById("chart");
    const chart = new Chart(ctx, {
      type: "line",
      data: {
        labels: [],
        datasets: [
          { label: "FL", borderColor: colors.fl, data: [], tension: 0.1, pointRadius: 0, borderWidth: 1.5 },
          { label: "BL", borderColor: colors.bl, data: [], tension: 0.1, pointRadius: 0, borderWidth: 1.5 },
          { label: "BR", borderColor: colors.br, data: [], tension: 0.1, pointRadius: 0, borderWidth: 1.5 },
          { label: "FR", borderColor: colors.fr, data: [], tension: 0.1, pointRadius: 0, borderWidth: 1.5 },
        ],
      },
      options: {
        animation: false,
        responsive: true,
        maintainAspectRatio: true,
        interaction: { mode: "index", intersect: false },
        scales: {
          x: { ticks: { color: "#888" }, grid: { color: "#333" }, title: { display: true, text: "sample #", color: "#888" } },
          y: { ticks: { color: "#888" }, grid: { color: "#333" }, title: { display: true, text: "raw", color: "#888" } },
        },
        plugins: { legend: { labels: { color: "#ccc" } } },
      },
    });

    async function poll() {
      try {
        const res = await fetch("/wheel-raw.json");
        const body = await res.json();
        const samples = body.samples || [];
        chart.data.labels = samples.map((_, i) => i);
        chart.data.datasets[0].data = samples.map(s => s.fl);
        chart.data.datasets[1].data = samples.map(s => s.bl);
        chart.data.datasets[2].data = samples.map(s => s.br);
        chart.data.datasets[3].data = samples.map(s => s.fr);
        chart.update("none");
        const last = samples[samples.length - 1];
        const meta = document.getElementById("meta");
        if (last) {
          meta.textContent = "samples: " + samples.length +
            " | latest FL=" + last.fl + " BL=" + last.bl + " BR=" + last.br + " FR=" + last.fr;
        } else {
          meta.textContent = "waiting for Wheel(raw) data…";
        }
      } catch (e) {
        document.getElementById("meta").textContent = "fetch error: " + e;
      }
    }
    setInterval(poll, 100);
    poll();
  </script>
</body>
</html>`
