import { useEffect, useMemo, useRef, useState } from 'react'
import ReactECharts from 'echarts-for-react'
import {
  Activity,
  Camera,
  Crosshair,
  Gauge,
  Map,
  Radio,
  Route,
  Satellite,
  Timer,
} from 'lucide-react'
import './App.css'

const HISTORY_LIMIT = 240

function App() {
  const [state, setState] = useState(null)
  const [connected, setConnected] = useState(false)
  const [history, setHistory] = useState([])
  const [launcherConfig, setLauncherConfig] = useState(null)
  const [launcherStatus, setLauncherStatus] = useState(null)
  const [launcherAvailable, setLauncherAvailable] = useState(false)
  const [launcherChecked, setLauncherChecked] = useState(false)

  useEffect(() => {
    let socket
    fetch('/api/launcher/config', { cache: 'no-store' })
      .then((res) => {
        if (!res.ok) throw new Error('launcher unavailable')
        return res.json()
      })
      .then((config) => {
        setLauncherConfig(config)
        setLauncherAvailable(true)
        return fetch('/api/launcher/status', { cache: 'no-store' })
      })
      .then((res) => res?.json?.())
      .then((status) => {
        if (status) setLauncherStatus(status)
        setLauncherChecked(true)
      })
      .catch(() => {
        setLauncherAvailable(false)
        setLauncherChecked(true)
      })

    try {
      const proto = window.location.protocol === 'https:' ? 'wss' : 'ws'
      socket = new WebSocket(`${proto}://${window.location.host}/ws/launcher`)
      socket.onmessage = (event) => {
        setLauncherAvailable(true)
        setLauncherChecked(true)
        setLauncherStatus(JSON.parse(event.data))
      }
    } catch {
      queueMicrotask(() => setLauncherAvailable(false))
    }
    return () => {
      if (socket) socket.close()
    }
  }, [])

  const liveApiBase = launcherAvailable ? launcherStatus?.live_api_url : ''
  const liveReady = launcherChecked && (!launcherAvailable || launcherStatus?.running)

  useEffect(() => {
    if (!liveReady) {
      const timer = window.setTimeout(() => {
        setConnected(false)
        setState(null)
        setHistory([])
      }, 0)
      return () => window.clearTimeout(timer)
    }

    let closed = false
    let socket
    let pollTimer

    const pushState = (next) => {
      setState(next)
      setHistory((items) => {
        const match = next.match || {}
        const pred = match.pred_ned || {}
        const truth = next.truth?.fresh ? next.truth : next.telemetry?.seed_ned || {}
        const sample = {
          t: Number(next.uptime_sec || 0),
          error: finite(match.independent_error_m) ? match.independent_error_m : match.error_m,
          inliers: match.inliers,
          good: match.good_count,
          predNorth: pred.north,
          predEast: pred.east,
          truthNorth: truth.north,
          truthEast: truth.east,
        }
        return [...items, sample].slice(-HISTORY_LIMIT)
      })
    }

    const startPolling = () => {
      if (pollTimer) return
      pollTimer = window.setInterval(async () => {
        try {
          const res = await fetch(apiUrl(liveApiBase, '/api/state'), { cache: 'no-store' })
          pushState(await res.json())
          setConnected(true)
        } catch {
          setConnected(false)
        }
      }, 1000)
    }

    const connect = () => {
      const proto = window.location.protocol === 'https:' ? 'wss' : 'ws'
      const wsUrl = liveApiBase
        ? `${liveApiBase.replace(/^http/, 'ws')}/ws/state`
        : `${proto}://${window.location.host}/ws/state`
      socket = new WebSocket(wsUrl)
      socket.onopen = () => setConnected(true)
      socket.onmessage = (event) => pushState(JSON.parse(event.data))
      socket.onerror = () => {
        setConnected(false)
        startPolling()
      }
      socket.onclose = () => {
        setConnected(false)
        if (!closed) startPolling()
      }
    }

    connect()
    return () => {
      closed = true
      if (socket) socket.close()
      if (pollTimer) window.clearInterval(pollTimer)
    }
  }, [liveApiBase, liveReady])

  const match = state?.match || {}
  const telemetry = state?.telemetry || {}
  const truth = state?.truth?.fresh ? state.truth : telemetry.seed_ned || {}
  const pred = match.pred_ned || {}
  const error = finite(match.independent_error_m) ? match.independent_error_m : match.error_m
  const status = match.status || (launcherAvailable && !launcherStatus?.running ? 'STOPPED' : 'IDLE')
  const reject = match.reject?.reason || match.reject_reason
  const isOk = status === 'OK'
  const isBad = status === 'REJECTED' || state?.video_status === 'open_failed'

  const errorChart = useMemo(() => makeErrorChart(history), [history])
  const matchChart = useMemo(() => makeMatchChart(history), [history])

  return (
    <div className="shell">
      <header className="topbar">
        <div>
          <h1>SAU SIFT Nav Dashboard</h1>
          <p>{state?.map ? `${state.map.map_w} x ${state.map.map_h} px · ${fmt(state.map.meters_per_px, 5)} m/px · NED ${state.map.ned_mode}` : 'launcher ready · configure and start estimation'}</p>
        </div>
        <div className="top-actions">
          <StatusPill label={connected ? 'live' : 'offline'} tone={connected ? 'good' : 'bad'} icon={<Radio size={16} />} />
          <StatusPill label={status.toLowerCase()} tone={isOk ? 'good' : isBad ? 'bad' : 'warn'} icon={<Activity size={16} />} />
        </div>
      </header>

      <main className="layout">
        <section className="map-panel">
          <MapOverlay state={state} history={history} liveApiBase={liveApiBase} />
        </section>

        <aside className="side">
          {launcherAvailable && (
            <LauncherPanel
              launcherConfig={launcherConfig}
              launcherStatus={launcherStatus}
              onStatus={setLauncherStatus}
            />
          )}
          <section className="video-panel">
            <div className="panel-heading">
              <Camera size={17} />
              <span>Kamera</span>
            </div>
            {state ? (
              <img src={apiUrl(liveApiBase, '/api/frame.mjpg')} alt="" />
            ) : (
              <div className="video-empty">estimator stopped</div>
            )}
          </section>

          <section className="metrics">
            <Metric icon={<Crosshair size={18} />} label="Tahmin N/E" value={`${fmt(pred.north)} / ${fmt(pred.east)} m`} />
            <Metric icon={<Satellite size={18} />} label="Gercek N/E" value={`${fmt(truth.north)} / ${fmt(truth.east)} m`} sub={truth.source || state?.truth?.source || '-'} />
            <Metric icon={<Gauge size={18} />} label="Hata" value={finite(error) ? `${fmt(error)} m` : '-'} sub={reject ? `reject: ${reject}` : 'ground truth / telemetry'} />
            <Metric icon={<Route size={18} />} label="Inlier / Good" value={`${safe(match.inliers)} / ${safe(match.good_count)}`} sub={match.tile_img || '-'} />
            <Metric icon={<Timer size={18} />} label="Sure / FPS" value={`${fmt(match.duration_sec, 3)} s`} sub={finite(match.duration_sec) && match.duration_sec > 0 ? `${fmt(1 / match.duration_sec, 1)} FPS` : '-'} />
            <Metric icon={<Radio size={18} />} label="Vision TX" value={state?.vision_tx?.status || '-'} sub={`count ${state?.vision_tx?.sent_count || 0}`} />
          </section>
        </aside>

        <section className="charts">
          <ChartPanel title="Konum Hatası">
            <ReactECharts option={errorChart} notMerge lazyUpdate />
          </ChartPanel>
          <ChartPanel title="Eşleşme Kalitesi">
            <ReactECharts option={matchChart} notMerge lazyUpdate />
          </ChartPanel>
        </section>

        <section className="log-panel">
          <div className="panel-heading">
            <Map size={17} />
            <span>Durum</span>
          </div>
          <pre>{buildLog(state, launcherStatus)}</pre>
        </section>
      </main>
    </div>
  )
}

function LauncherPanel({ launcherConfig, launcherStatus, onStatus }) {
  const presets = launcherConfig?.presets || []
  const defaults = launcherConfig?.defaults || {}
  const fields = launcherConfig?.fields || []
  const [presetId, setPresetId] = useState('observe_recommended')
  const [overrides, setOverrides] = useState({})
  const [extraArgs, setExtraArgs] = useState('')
  const [busy, setBusy] = useState(false)
  const [gimbalBusy, setGimbalBusy] = useState(false)
  const [gimbalMessage, setGimbalMessage] = useState('')
  const [dummyBusy, setDummyBusy] = useState(false)
  const [dummyMessage, setDummyMessage] = useState('')
  const [sourceBusy, setSourceBusy] = useState(false)
  const [sourceMessage, setSourceMessage] = useState('')

  const selectedPreset = presets.find((item) => item.id === presetId)
  const running = Boolean(launcherStatus?.running)
  const dummyVision = launcherStatus?.dummy_vision || {}
  const dummyRunning = Boolean(dummyVision.running)

  const update = (name, value, type) => {
    setOverrides((current) => ({
      ...current,
      [name]: type === 'number' ? (value === '' ? '' : Number(value)) : value,
    }))
  }

  const selectPreset = (id) => {
    setPresetId(id)
    setOverrides({})
  }

  const start = async () => {
    setBusy(true)
    try {
      const res = await fetch('/api/launcher/start', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ preset_id: selectedPreset?.id || presetId, overrides, extra_args: extraArgs }),
      })
      const body = await res.json()
      if (!res.ok) throw new Error(body.detail || 'start failed')
      onStatus(body)
    } catch (err) {
      alert(`Baslatma hatasi: ${err.message}`)
    } finally {
      setBusy(false)
    }
  }

  const stop = async () => {
    setBusy(true)
    try {
      const res = await fetch('/api/launcher/stop', { method: 'POST' })
      onStatus(await res.json())
    } finally {
      setBusy(false)
    }
  }

  const pointGimbalDown = async () => {
    setGimbalBusy(true)
    setGimbalMessage('')
    const mavlink =
      launcherStatus?.config?.mavlink ||
      overrides.mavlink ||
      selectedPreset?.config?.mavlink ||
      defaults.mavlink
    try {
      const res = await fetch('/api/launcher/gimbal/down', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mavlink, pitch_deg: -90, roll_deg: 0, yaw_deg: 0 }),
      })
      const body = await res.json()
      if (!res.ok) throw new Error(body.detail || 'gimbal command failed')
      setGimbalMessage(body.ack_result_name || body.status || 'sent')
      const statusRes = await fetch('/api/launcher/status', { cache: 'no-store' })
      onStatus(await statusRes.json())
    } catch (err) {
      setGimbalMessage(`error: ${err.message}`)
    } finally {
      setGimbalBusy(false)
    }
  }

  const selectedMavlink = () => (
    launcherStatus?.config?.mavlink ||
    overrides.mavlink ||
    selectedPreset?.config?.mavlink ||
    defaults.mavlink
  )

  const startDummyVision = async () => {
    setDummyBusy(true)
    setDummyMessage('')
    try {
      const res = await fetch('/api/launcher/dummy-vision/start', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          mavlink: selectedMavlink(),
          rate_hz: 10,
          x: 0,
          y: 0,
          z: 0,
          roll: 0,
          pitch: 0,
          yaw: 0,
          reset_counter: 0,
        }),
      })
      const body = await res.json()
      if (!res.ok) throw new Error(body.detail || 'dummy VPE failed')
      onStatus(body)
      setDummyMessage('dummy VPE started @ 10Hz')
    } catch (err) {
      setDummyMessage(`error: ${err.message}`)
    } finally {
      setDummyBusy(false)
    }
  }

  const stopDummyVision = async () => {
    setDummyBusy(true)
    setDummyMessage('')
    try {
      const res = await fetch('/api/launcher/vpe/off', { method: 'POST' })
      const body = await res.json()
      if (!res.ok) throw new Error(body.detail || 'dummy VPE stop failed')
      onStatus(body)
      setDummyMessage('VPE off')
    } catch (err) {
      setDummyMessage(`error: ${err.message}`)
    } finally {
      setDummyBusy(false)
    }
  }

  const switchSourceSet = async (sourceSet) => {
    setSourceBusy(true)
    setSourceMessage('')
    try {
      const res = await fetch('/api/launcher/source-set', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mavlink: selectedMavlink(), source_set: sourceSet }),
      })
      const body = await res.json()
      if (!res.ok) throw new Error(body.detail || 'source-set failed')
      setSourceMessage(`source set ${sourceSet}: ${body.ack_result_name || body.status || 'sent'}`)
      const statusRes = await fetch('/api/launcher/status', { cache: 'no-store' })
      onStatus(await statusRes.json())
    } catch (err) {
      setSourceMessage(`error: ${err.message}`)
    } finally {
      setSourceBusy(false)
    }
  }

  return (
    <section className="launcher-panel">
      <div className="panel-heading">
        <Radio size={17} />
        <span>Baslatma</span>
      </div>
      <div className="launcher-body">
        <label>
          <span>Preset</span>
          <select value={presetId} onChange={(event) => selectPreset(event.target.value)} disabled={running}>
            {presets.map((preset) => (
              <option key={preset.id} value={preset.id}>{preset.name}</option>
            ))}
          </select>
        </label>
        {selectedPreset && <p className="preset-desc">{selectedPreset.description}</p>}
        <div className="config-grid">
          {fields.map((field) => (
            <ConfigField
              key={field.name}
              field={field}
              value={overrides[field.name] ?? selectedPreset?.config?.[field.name] ?? defaults[field.name] ?? ''}
              disabled={running}
              onChange={update}
            />
          ))}
        </div>
        <label>
          <span>Ek CLI argumanlari</span>
          <input
            value={extraArgs}
            onChange={(event) => setExtraArgs(event.target.value)}
            placeholder="orn: --nfeatures 900 --ratio 0.72"
            disabled={running}
          />
        </label>
        <div className="launcher-actions">
          <button onClick={start} disabled={busy || running}>Baslat</button>
          <button className="secondary" onClick={stop} disabled={busy || !running}>Durdur</button>
          <button className="neutral" onClick={() => switchSourceSet(1)} disabled={sourceBusy}>SRC 1 GPS</button>
          <button className="quaternary" onClick={() => switchSourceSet(2)} disabled={sourceBusy}>SRC 2 VISION</button>
          <button className="tertiary" onClick={pointGimbalDown} disabled={gimbalBusy}>Gimbal Down</button>
          <button className="quaternary" onClick={startDummyVision} disabled={dummyBusy || dummyRunning}>Dummy VPE ON</button>
          <button className="secondary" onClick={stopDummyVision} disabled={dummyBusy}>VPE OFF</button>
          <StatusPill label={running ? `pid ${launcherStatus.pid}` : 'stopped'} tone={running ? 'good' : 'warn'} icon={<Activity size={15} />} />
          <StatusPill label={dummyRunning ? `dummy ${dummyVision.sent_count || 0}` : 'dummy off'} tone={dummyRunning ? 'good' : 'warn'} icon={<Radio size={15} />} />
        </div>
        {gimbalMessage && <p className="gimbal-message">{gimbalMessage}</p>}
        {dummyMessage && <p className="gimbal-message">{dummyMessage}</p>}
        {sourceMessage && <p className="gimbal-message">{sourceMessage}</p>}
        <div className="launcher-links">
          <span>Live API: {launcherStatus?.live_api_url || '-'}</span>
          <span>Eski UI: {launcherStatus?.old_dashboard_url || '-'}</span>
        </div>
        <pre className="launcher-log">{(launcherStatus?.log_tail || []).slice(-8).join('\n')}</pre>
      </div>
    </section>
  )
}

function ConfigField({ field, value, disabled, onChange }) {
  if (field.type === 'boolean') {
    return (
      <label className="check-field">
        <input
          type="checkbox"
          checked={Boolean(value)}
          disabled={disabled}
          onChange={(event) => onChange(field.name, event.target.checked, field.type)}
        />
        <span>{field.label}</span>
      </label>
    )
  }
  if (field.type === 'select') {
    return (
      <label>
        <span>{field.label}</span>
        <select value={value} disabled={disabled} onChange={(event) => onChange(field.name, event.target.value, field.type)}>
          {(field.options || []).map((option) => <option key={option} value={option}>{option}</option>)}
        </select>
      </label>
    )
  }
  return (
    <label>
      <span>{field.label}</span>
      <input
        type={field.type === 'number' ? 'number' : 'text'}
        value={value}
        disabled={disabled}
        onChange={(event) => onChange(field.name, event.target.value, field.type)}
      />
    </label>
  )
}

function MapOverlay({ state, history, liveApiBase }) {
  const imgRef = useRef(null)
  const canvasRef = useRef(null)

  useEffect(() => {
    const image = imgRef.current
    const canvas = canvasRef.current
    if (!image || !canvas || !state?.map) return

    const draw = () => {
      const rect = image.getBoundingClientRect()
      const dpr = window.devicePixelRatio || 1
      canvas.style.width = `${rect.width}px`
      canvas.style.height = `${rect.height}px`
      canvas.width = Math.max(1, Math.round(rect.width * dpr))
      canvas.height = Math.max(1, Math.round(rect.height * dpr))
      const ctx = canvas.getContext('2d')
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
      ctx.clearRect(0, 0, rect.width, rect.height)

      const sx = rect.width / state.map.map_w
      const sy = rect.height / state.map.map_h
      const toCanvas = (point) => [point[0] * sx, point[1] * sy]
      const match = state.match || {}
      const truthPx = state.truth?.global_px_1x || state.telemetry?.global_px_1x

      if (history.length > 1) {
        drawTrail(ctx, history, state.map, rect, 'pred', 'rgba(240, 96, 96, 0.86)')
        drawTrail(ctx, history, state.map, rect, 'truth', 'rgba(75, 170, 255, 0.82)')
      }

      if (match.tile_bbox_1x) {
        const b = match.tile_bbox_1x
        ctx.save()
        ctx.strokeStyle = match.status === 'REJECTED' ? '#f06060' : '#f1c75b'
        ctx.lineWidth = 3
        ctx.strokeRect(b[0] * sx, b[1] * sy, (b[2] - b[0]) * sx, (b[3] - b[1]) * sy)
        ctx.restore()
      }

      if (match.frame_quad_1x?.length) {
        drawPolygon(ctx, match.frame_quad_1x.map(toCanvas), '#49d5d0', 'rgba(73, 213, 208, 0.16)')
      }

      if (truthPx) {
        const [x, y] = toCanvas(truthPx)
        drawCircle(ctx, x, y, '#4baaff')
      }
      if (match.global_px_1x) {
        const [x, y] = toCanvas(match.global_px_1x)
        drawCross(ctx, x, y, '#f06060')
      }
      if (truthPx && match.global_px_1x) {
        const a = toCanvas(truthPx)
        const b = toCanvas(match.global_px_1x)
        ctx.save()
        ctx.strokeStyle = 'rgba(255,255,255,.72)'
        ctx.lineWidth = 2
        ctx.setLineDash([7, 5])
        ctx.beginPath()
        ctx.moveTo(a[0], a[1])
        ctx.lineTo(b[0], b[1])
        ctx.stroke()
        ctx.restore()
      }
    }

    draw()
    image.addEventListener('load', draw)
    window.addEventListener('resize', draw)
    return () => {
      image.removeEventListener('load', draw)
      window.removeEventListener('resize', draw)
    }
  }, [state, history])

  if (!state?.map) {
    return (
      <div className="map-wrap map-empty">
        <div>configure a preset and press Baslat</div>
      </div>
    )
  }

  return (
    <div className="map-wrap">
      <img ref={imgRef} src={apiUrl(liveApiBase, '/api/map.jpg')} alt="" />
      <canvas ref={canvasRef} />
      <div className="legend">
        <span><i className="red" />Tahmin</span>
        <span><i className="blue" />Gercek</span>
        <span><i className="gold" />Patch</span>
        <span><i className="cyan" />Kamera izi</span>
      </div>
    </div>
  )
}

function Metric({ icon, label, value, sub }) {
  return (
    <div className="metric">
      <div className="metric-icon">{icon}</div>
      <div>
        <span>{label}</span>
        <strong>{value}</strong>
        {sub && <small>{sub}</small>}
      </div>
    </div>
  )
}

function StatusPill({ icon, label, tone }) {
  return <div className={`pill ${tone}`}>{icon}<span>{label}</span></div>
}

function ChartPanel({ title, children }) {
  return (
    <section className="chart-panel">
      <div className="panel-heading"><Activity size={17} /><span>{title}</span></div>
      {children}
    </section>
  )
}

function makeErrorChart(history) {
  return {
    grid: { left: 44, right: 16, top: 18, bottom: 30 },
    tooltip: { trigger: 'axis' },
    xAxis: { type: 'category', data: history.map((p) => fmt(p.t, 1)), axisLabel: { color: '#99a39a' } },
    yAxis: { type: 'value', name: 'm', nameTextStyle: { color: '#99a39a' }, axisLabel: { color: '#99a39a' }, splitLine: { lineStyle: { color: '#293029' } } },
    series: [{ type: 'line', data: history.map((p) => finite(p.error) ? p.error : null), smooth: true, showSymbol: false, lineStyle: { color: '#f06060', width: 2 } }],
    backgroundColor: 'transparent',
  }
}

function makeMatchChart(history) {
  return {
    grid: { left: 44, right: 16, top: 18, bottom: 30 },
    tooltip: { trigger: 'axis' },
    legend: { data: ['Inlier', 'Good'], textStyle: { color: '#cdd6cc' }, top: 0 },
    xAxis: { type: 'category', data: history.map((p) => fmt(p.t, 1)), axisLabel: { color: '#99a39a' } },
    yAxis: { type: 'value', axisLabel: { color: '#99a39a' }, splitLine: { lineStyle: { color: '#293029' } } },
    series: [
      { name: 'Inlier', type: 'line', data: history.map((p) => finite(p.inliers) ? p.inliers : null), smooth: true, showSymbol: false, lineStyle: { color: '#6bdc8f', width: 2 } },
      { name: 'Good', type: 'line', data: history.map((p) => finite(p.good) ? p.good : null), smooth: true, showSymbol: false, lineStyle: { color: '#f1c75b', width: 2 } },
    ],
    backgroundColor: 'transparent',
  }
}

function drawTrail(ctx, history, map, rect, kind, color) {
  const points = history
    .map((p) => {
      const north = kind === 'pred' ? p.predNorth : p.truthNorth
      const east = kind === 'pred' ? p.predEast : p.truthEast
      if (!finite(north) || !finite(east)) return null
      const px = nedToPixel(map, north, east)
      if (!px) return null
      return [px[0] * rect.width / map.map_w, px[1] * rect.height / map.map_h]
    })
    .filter(Boolean)
  if (points.length < 2) return
  ctx.save()
  ctx.strokeStyle = color
  ctx.lineWidth = 2
  ctx.beginPath()
  ctx.moveTo(points[0][0], points[0][1])
  for (const p of points.slice(1)) ctx.lineTo(p[0], p[1])
  ctx.stroke()
  ctx.restore()
}

function nedToPixel(map, north, east) {
  const mpp = map.meters_per_px
  if (!finite(mpp) || !mpp) return null
  if (map.ned_mode === 'enu') return [map.map_w / 2 + east / mpp, map.map_h / 2 - north / mpp]
  if (map.ned_mode === 'xy') return [map.map_w / 2 + north / mpp, map.map_h / 2 - east / mpp]
  if (map.ned_mode === 'neg_enu') return [map.map_w / 2 - east / mpp, map.map_h / 2 + north / mpp]
  return [map.map_w / 2 - north / mpp, map.map_h / 2 + east / mpp]
}

function drawPolygon(ctx, points, stroke, fill) {
  ctx.save()
  ctx.strokeStyle = stroke
  ctx.fillStyle = fill
  ctx.lineWidth = 2
  ctx.beginPath()
  ctx.moveTo(points[0][0], points[0][1])
  for (const p of points.slice(1)) ctx.lineTo(p[0], p[1])
  ctx.closePath()
  ctx.fill()
  ctx.stroke()
  ctx.restore()
}

function drawCircle(ctx, x, y, color) {
  ctx.save()
  ctx.strokeStyle = '#ffffff'
  ctx.lineWidth = 5
  ctx.beginPath()
  ctx.arc(x, y, 9, 0, Math.PI * 2)
  ctx.stroke()
  ctx.strokeStyle = color
  ctx.lineWidth = 3
  ctx.stroke()
  ctx.restore()
}

function drawCross(ctx, x, y, color) {
  ctx.save()
  ctx.strokeStyle = color
  ctx.lineWidth = 4
  ctx.lineCap = 'round'
  ctx.beginPath()
  ctx.moveTo(x - 13, y - 13)
  ctx.lineTo(x + 13, y + 13)
  ctx.moveTo(x + 13, y - 13)
  ctx.lineTo(x - 13, y + 13)
  ctx.stroke()
  ctx.restore()
}

function buildLog(state, launcherStatus) {
  if (!state) {
    return launcherStatus?.running ? 'waiting for live state' : 'estimator stopped; choose a preset and press Baslat'
  }
  const match = state.match || {}
  const search = match.search || {}
  const tx = state.vision_tx || {}
  const truth = state.truth || {}
  const telem = state.telemetry || {}
  const lines = [
    `video: ${state.video_status} frame=${state.frame?.id ?? '-'} age=${fmt(state.frame?.age_sec, 1)}s`,
    `match: ${match.status || '-'} tile=${match.tile_img || '-'} scale=${match.scale_name || '-'} age=${fmt(state.match_age_sec, 1)}s`,
    `search: ${search.source || '-'} radius=${fmt(search.radius_m)}m scanned=${safe(match.tiles_scanned)} considered=${safe(match.tiles_considered)}`,
    `vision: ${tx.status || '-'} mode=${tx.publish_mode || '-'} sent=${tx.sent_count || 0}`,
    `mavlink: ${telem.status || '-'} last=${telem.last_message || '-'}`,
    `truth: ${truth.status || '-'} source=${truth.source || '-'} fresh=${truth.fresh || false}`,
  ]
  if (match.frame_quad_metrics) {
    lines.push(`footprint: ${fmt(match.frame_quad_metrics.width_m, 1)} x ${fmt(match.frame_quad_metrics.height_m, 1)} m`)
  }
  if (state.errors?.length) {
    lines.push('')
    lines.push(...state.errors.map((err) => `ERR ${err}`))
  }
  return lines.join('\n')
}

function finite(value) {
  return Number.isFinite(Number(value))
}

function fmt(value, digits = 2) {
  return finite(value) ? Number(value).toFixed(digits) : '-'
}

function safe(value) {
  return value ?? '-'
}

function apiUrl(base, path) {
  if (!base) return path
  return `${base}${path}`
}

export default App
