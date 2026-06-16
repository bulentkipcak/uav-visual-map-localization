from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import urlparse

from .state import SharedState


INDEX_HTML = r"""<!doctype html>
<html lang="tr">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SAU SIFT Nav</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #151716;
      --panel: #202421;
      --line: #3d453f;
      --text: #edf3ed;
      --muted: #aeb9ad;
      --red: #ff5964;
      --blue: #49a6ff;
      --green: #5fe08f;
      --gold: #f1c75b;
      --cyan: #52d6d2;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      letter-spacing: 0;
    }
    .app { min-height: 100vh; display: grid; grid-template-columns: minmax(0, 1fr) 380px; }
    main { min-width: 0; display: grid; grid-template-rows: auto minmax(0, 1fr); border-right: 1px solid var(--line); }
    header {
      height: 56px; display: flex; align-items: center; justify-content: space-between;
      padding: 0 18px; background: #1d211e; border-bottom: 1px solid var(--line);
    }
    h1 { font-size: 17px; margin: 0; font-weight: 680; }
    .status-row { display: flex; gap: 10px; align-items: center; color: var(--muted); font-size: 13px; }
    .dot { width: 10px; height: 10px; border-radius: 50%; background: var(--gold); box-shadow: 0 0 0 3px rgba(241,199,91,.13); }
    .dot.ok { background: var(--green); box-shadow: 0 0 0 3px rgba(95,224,143,.14); }
    .dot.bad { background: var(--red); box-shadow: 0 0 0 3px rgba(255,89,100,.14); }
    .map-wrap { position: relative; min-height: 0; overflow: auto; background: #0e100f; }
    #mapImage { display: block; width: 100%; height: auto; user-select: none; }
    #overlay { position: absolute; inset: 0 auto auto 0; pointer-events: none; }
    aside { min-width: 0; background: var(--panel); display: grid; grid-template-rows: 220px auto minmax(0, 1fr); overflow: hidden; }
    .video { background: #0d0f0e; border-bottom: 1px solid var(--line); display: flex; align-items: center; justify-content: center; min-width: 0; min-height: 0; }
    #videoImage { max-width: 100%; max-height: 100%; object-fit: contain; }
    .section { padding: 14px 16px; border-bottom: 1px solid var(--line); }
    .section h2 { margin: 0 0 10px; font-size: 13px; color: var(--muted); font-weight: 650; text-transform: uppercase; }
    .kv { display: grid; grid-template-columns: 142px minmax(0, 1fr); gap: 7px 10px; font-size: 13px; line-height: 1.3; }
    .k { color: var(--muted); }
    .v { color: var(--text); overflow-wrap: anywhere; font-variant-numeric: tabular-nums; }
    .legend { display: flex; gap: 12px; flex-wrap: wrap; padding-top: 10px; font-size: 12px; color: var(--muted); }
    .swatch { display: inline-block; width: 10px; height: 10px; margin-right: 5px; border-radius: 2px; vertical-align: -1px; }
    .log { overflow: auto; padding: 14px 16px; font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; font-size: 12px; color: #cdd6cc; white-space: pre-wrap; }
    @media (max-width: 980px) {
      .app { grid-template-columns: 1fr; }
      main { border-right: 0; }
      aside { grid-template-rows: 190px auto auto; }
    }
  </style>
</head>
<body>
  <div class="app">
    <main>
      <header>
        <h1>SAU SIFT Nav</h1>
        <div class="status-row"><span id="dot" class="dot"></span><span id="topStatus">Basliyor</span></div>
      </header>
      <div id="mapWrap" class="map-wrap">
        <img id="mapImage" src="/map/preview.jpg" alt="">
        <canvas id="overlay"></canvas>
      </div>
    </main>
    <aside>
      <div class="video"><img id="videoImage" src="/video/latest.jpg" alt=""></div>
      <div class="section">
        <h2>Konum</h2>
        <div class="kv">
          <div class="k">Tahmin N/E</div><div id="predNe" class="v">-</div>
          <div class="k">Gercek N/E</div><div id="trueNe" class="v">-</div>
          <div class="k">Hata</div><div id="err" class="v">-</div>
          <div class="k">Vision TX</div><div id="visionTx" class="v">-</div>
          <div class="k">Arama</div><div id="search" class="v">-</div>
          <div class="k">Tile</div><div id="tile" class="v">-</div>
          <div class="k">Inliers</div><div id="inliers" class="v">-</div>
          <div class="k">Sure</div><div id="duration" class="v">-</div>
          <div class="k">Video</div><div id="videoStatus" class="v">-</div>
          <div class="k">MAVLink</div><div id="mavStatus" class="v">-</div>
        </div>
        <div class="legend">
          <span><span class="swatch" style="background: var(--red)"></span>Tahmin</span>
          <span><span class="swatch" style="background: var(--blue)"></span>Gercek</span>
          <span><span class="swatch" style="background: var(--gold)"></span>Tile</span>
          <span><span class="swatch" style="background: var(--cyan)"></span>Kamera izi</span>
        </div>
      </div>
      <div id="log" class="log"></div>
    </aside>
  </div>
  <script>
    const mapImage = document.getElementById('mapImage');
    const overlay = document.getElementById('overlay');
    const ctx = overlay.getContext('2d');
    const videoImage = document.getElementById('videoImage');

    function f(x, n = 2) { return Number.isFinite(x) ? x.toFixed(n) : '-'; }
    function age(x) { return Number.isFinite(x) ? `${x.toFixed(1)}s` : '-'; }
    function setText(id, value) { document.getElementById(id).textContent = value; }

    function syncCanvas() {
      const rect = mapImage.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      overlay.style.width = `${rect.width}px`;
      overlay.style.height = `${rect.height}px`;
      overlay.width = Math.max(1, Math.round(rect.width * dpr));
      overlay.height = Math.max(1, Math.round(rect.height * dpr));
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      return { width: rect.width, height: rect.height };
    }

    function drawCross(x, y, color, size = 12, width = 3) {
      ctx.save(); ctx.strokeStyle = color; ctx.lineWidth = width; ctx.lineCap = 'round';
      ctx.beginPath(); ctx.moveTo(x - size, y - size); ctx.lineTo(x + size, y + size);
      ctx.moveTo(x + size, y - size); ctx.lineTo(x - size, y + size); ctx.stroke(); ctx.restore();
    }

    function drawCircle(x, y, color) {
      ctx.save(); ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 5;
      ctx.beginPath(); ctx.arc(x, y, 9, 0, Math.PI * 2); ctx.stroke();
      ctx.strokeStyle = color; ctx.lineWidth = 3; ctx.stroke(); ctx.restore();
    }

    function drawPolygon(points, color, fill) {
      if (!points || points.length < 3) return;
      ctx.save(); ctx.strokeStyle = color; ctx.fillStyle = fill; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(points[0][0], points[0][1]);
      for (let i = 1; i < points.length; i++) ctx.lineTo(points[i][0], points[i][1]);
      ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.restore();
    }

    function render(state) {
      const rect = syncCanvas();
      ctx.clearRect(0, 0, rect.width, rect.height);
      const sx = rect.width / state.map.map_w;
      const sy = rect.height / state.map.map_h;
      const toCanvas = p => [p[0] * sx, p[1] * sy];
      const match = state.match || {};
      const telem = state.telemetry || {};
      const truthState = state.truth || {};
      const tx = state.vision_tx || {};
      const align = state.vision_alignment || {};
      const truthPx = truthState.global_px_1x || telem.global_px_1x;

      if (match.tile_bbox_1x) {
        const b = match.tile_bbox_1x;
        ctx.save(); ctx.strokeStyle = match.status === 'REJECTED' ? '#ff5964' : '#f1c75b'; ctx.lineWidth = 3;
        ctx.strokeRect(b[0] * sx, b[1] * sy, (b[2] - b[0]) * sx, (b[3] - b[1]) * sy);
        ctx.restore();
      }
      if (match.frame_quad_1x) drawPolygon(match.frame_quad_1x.map(toCanvas), '#52d6d2', 'rgba(82,214,210,.14)');
      if (truthPx) { const p = toCanvas(truthPx); drawCircle(p[0], p[1], '#49a6ff'); }
      if (match.global_px_1x) { const p = toCanvas(match.global_px_1x); drawCross(p[0], p[1], '#ff5964', 13, 4); }
      if (truthPx && match.global_px_1x) {
        const a = toCanvas(truthPx), b = toCanvas(match.global_px_1x);
        ctx.save(); ctx.strokeStyle = 'rgba(255,255,255,.72)'; ctx.lineWidth = 2; ctx.setLineDash([7,5]);
        ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke(); ctx.restore();
      }

      const ok = match.status === 'OK';
      const hasPose = ok || match.status === 'REJECTED';
      const dot = document.getElementById('dot');
      dot.className = ok ? 'dot ok' : (match.status === 'REJECTED' || state.video_status === 'open_failed' ? 'dot bad' : 'dot');
      const reject = match.reject ? ` | reject ${match.reject.reason}` : '';
      setText('topStatus', `${match.status || 'IDLE'}${reject} | frame ${state.frame.id} | match age ${age(state.match_age_sec)}`);

      const pred = match.pred_ned || {};
      const truth = Number.isFinite(truthState.north) ? truthState : (telem.seed_ned || {});
      const local = telem.local_ned || {};
      const globalPosition = telem.global_position || {};
      const search = match.search || {};
      setText('predNe', hasPose ? `${f(pred.north)} / ${f(pred.east)} m` : '-');
      setText('trueNe', Number.isFinite(truth.north) ? `${f(truth.north)} / ${f(truth.east)} m (${truth.source || '-'}, ${age(truth.age_sec)})` : '-');
      setText('err', Number.isFinite(match.error_m) ? `${f(match.error_m)} m` : '-');
      setText('visionTx', `${tx.status || '-'} ${tx.publish_mode || 'rate'} ${tx.rate_hz ? '@ ' + tx.rate_hz + 'Hz' : ''} count=${tx.sent_count || 0}`);
      setText('search', `${search.source || '-'} r=${f(search.radius_m)}m tiles=${match.tiles_scanned || '-'}`);
      setText('tile', hasPose ? `${match.scale_name} ${match.tile_img}` : '-');
      setText('inliers', hasPose ? `${match.inliers} / good ${match.good_count}` : '-');
      setText('duration', Number.isFinite(match.duration_sec) ? `${f(match.duration_sec, 3)} s` : '-');
      setText('videoStatus', `${state.video_status} | age ${age(state.frame.age_sec)}`);
      const localText = local.age_sec !== undefined ? `local:${local.fresh ? '' : 'stale '}${age(local.age_sec)}` : 'local:-';
      const gpsText = globalPosition.age_sec !== undefined ? `gps:${globalPosition.fresh ? '' : 'stale '}${age(globalPosition.age_sec)}` : 'gps:-';
      setText('mavStatus', `${telem.status || '-'} | ${telem.last_message || '-'} | ${localText} ${gpsText}`);

      const lines = [];
      lines.push(`map: ${state.map.map_w}x${state.map.map_h}, ${state.map.meters_per_px.toFixed(5)} m/px, ned=${state.map.ned_mode}`);
      lines.push(`vision tx: ${tx.status || '-'} mode=${tx.publish_mode || 'rate'} ts=${tx.vision_timestamp_source || tx.timestamp_source || '-'} extra=${f(tx.pose_extra_predict_sec ?? tx.extra_predict_sec ?? 0, 2)}s source=${tx.source || '-'} speed=${tx.speed_source || '-'} age=${age(tx.age_sec)} reset=${tx.reset_counter || 0}`);
      if (align && align.source) lines.push(`vision align: ${align.source} seed=${align.seed_source || '-'} offset=${f(align.offset_north)}/${f(align.offset_east)}m age=${age(align.age_sec)}`);
      if (state.nav_gates) lines.push(`gates: telemetry=${state.nav_gates.telemetry_position_gate ? state.nav_gates.max_nav_error_m + 'm' : 'off'} source=${state.nav_gates.telemetry_seed_source || 'auto'} max_age=${state.nav_gates.telemetry_max_age_sec}s jump=${state.nav_gates.max_nav_jump_m}m inliers=${state.nav_gates.min_nav_inliers}`);
      if (match.reject) lines.push(`reject: ${match.reject.reason} ${JSON.stringify(match.reject)}`);
      if (match.per_scale) {
        for (const s of match.per_scale) {
          lines.push(`${s.scale_name || '-'} ${s.status} ${s.tile_img || ''} inl=${s.inliers || '-'} tiles=${s.tiles_scanned || '-'} t=${Number.isFinite(s.duration_sec) ? s.duration_sec.toFixed(3) : '-'}`);
        }
      }
      if (match.frame_quad_metrics) {
        const q = match.frame_quad_metrics;
        lines.push(`camera footprint: ${f(q.width_m, 1)} x ${f(q.height_m, 1)}m area=${f(q.area_m2, 0)}m2`);
      }
      if (truthState && truthState.source) lines.push(`gazebo truth: ${truthState.status || '-'} age=${age(truthState.age_sec)} N/E=${f(truthState.north)}/${f(truthState.east)}m pose_age=${age(truthState.pose_age_sec)}`);
      if (telem.seed_ned) lines.push(`truth seed: ${telem.seed_ned.source} age=${age(telem.seed_ned.age_sec)} N/E=${f(telem.seed_ned.north)}/${f(telem.seed_ned.east)}m`);
      if (telem.global_position) lines.push(`gps: ${telem.global_position.lat.toFixed(7)}, ${telem.global_position.lon.toFixed(7)} rel_alt=${f(telem.global_position.relative_alt_m)}m age=${age(telem.global_position.age_sec)} fresh=${telem.global_position.fresh}`);
      if (telem.local_ned) lines.push(`local: N/E=${f(telem.local_ned.north)}/${f(telem.local_ned.east)}m age=${age(telem.local_ned.age_sec)} fresh=${telem.local_ned.fresh}`);
      if (state.errors && state.errors.length) { lines.push(''); lines.push(...state.errors.map(e => `ERR ${e}`)); }
      document.getElementById('log').textContent = lines.join('\n');
    }

    async function tick() {
      try {
        const res = await fetch('/api/state', { cache: 'no-store' });
        const state = await res.json();
        render(state);
        if (state.frame && state.frame.id) videoImage.src = `/video/latest.jpg?frame=${state.frame.id}`;
      } catch (err) {
        setText('topStatus', `Baglanti hatasi: ${err}`);
      }
    }
    mapImage.addEventListener('load', tick);
    window.addEventListener('resize', tick);
    setInterval(tick, 1000);
    tick();
  </script>
</body>
</html>
"""


def make_handler(state: SharedState, preview_map: str) -> type:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *args: Any) -> None:
            return

        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path == "/":
                self._send_bytes(INDEX_HTML.encode("utf-8"), "text/html; charset=utf-8")
                return
            if path == "/api/state":
                payload = json.dumps(state.snapshot(), ensure_ascii=False).encode("utf-8")
                self._send_bytes(payload, "application/json; charset=utf-8")
                return
            if path == "/map/preview.jpg":
                self._send_file(preview_map, "image/jpeg")
                return
            if path == "/video/latest.jpg":
                jpg = state.get_jpeg()
                if jpg is None:
                    self.send_response(204)
                    self.end_headers()
                else:
                    self._send_bytes(jpg, "image/jpeg")
                return
            self.send_response(404)
            self.end_headers()

        def _send_file(self, path: str, content_type: str) -> None:
            try:
                with open(path, "rb") as f:
                    data = f.read()
            except FileNotFoundError:
                self.send_response(404)
                self.end_headers()
                return
            self._send_bytes(data, content_type)

        def _send_bytes(self, data: bytes, content_type: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

    return Handler


def make_server(host: str, port: int, state: SharedState, preview_map: str) -> ThreadingHTTPServer:
    return ThreadingHTTPServer((host, port), make_handler(state, preview_map))
