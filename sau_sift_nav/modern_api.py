from __future__ import annotations

import asyncio
import mimetypes
import threading
from pathlib import Path
from typing import Any, Optional

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles

from .state import SharedState


def create_app(state: SharedState, preview_map: str, static_dir: Optional[str] = None) -> FastAPI:
    app = FastAPI(title="SAU SIFT Nav API", version="0.1.0")
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    preview_path = Path(preview_map)

    @app.get("/api/state")
    def api_state() -> dict[str, Any]:
        return state.snapshot()

    @app.get("/api/status")
    def api_status() -> dict[str, Any]:
        return compact_status(state.snapshot())

    @app.get("/api/map.jpg")
    def api_map() -> FileResponse:
        media_type = mimetypes.guess_type(str(preview_path))[0] or "image/jpeg"
        return FileResponse(preview_path, media_type=media_type)

    @app.get("/api/frame.jpg")
    def api_frame() -> Response:
        jpg = state.get_jpeg()
        if jpg is None:
            return Response(status_code=204)
        return Response(content=jpg, media_type="image/jpeg", headers={"Cache-Control": "no-store"})

    @app.get("/api/frame.mjpg")
    def api_frame_stream() -> StreamingResponse:
        async def frames():
            last_frame_id = -1
            while True:
                frame_id, jpg = state.get_jpeg_with_id()
                if jpg is None or frame_id == last_frame_id:
                    await asyncio.sleep(0.03)
                    continue
                last_frame_id = frame_id
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Cache-Control: no-store\r\n"
                    b"Content-Length: "
                    + str(len(jpg)).encode("ascii")
                    + b"\r\n\r\n"
                    + jpg
                    + b"\r\n"
                )

        return StreamingResponse(
            frames(),
            media_type="multipart/x-mixed-replace; boundary=frame",
            headers={"Cache-Control": "no-store"},
        )

    @app.websocket("/ws/state")
    async def ws_state(websocket: WebSocket) -> None:
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(state.snapshot())
                await asyncio.sleep(0.25)
        except WebSocketDisconnect:
            return

    if static_dir:
        static_path = Path(static_dir)
        if static_path.exists():
            assets_dir = static_path / "assets"
            index_file = static_path / "index.html"
            if assets_dir.exists():
                app.mount("/assets", StaticFiles(directory=assets_dir), name="dashboard_assets")
            if index_file.exists():
                @app.get("/")
                def dashboard_index() -> FileResponse:
                    return FileResponse(index_file)

                @app.get("/{path:path}")
                def dashboard_fallback(path: str) -> FileResponse:
                    if path.startswith(("api/", "ws/", "assets/")):
                        raise HTTPException(status_code=404, detail="not found")
                    return FileResponse(index_file)

    return app


def compact_status(snapshot: dict[str, Any]) -> dict[str, Any]:
    match = snapshot.get("match") or {}
    telemetry = snapshot.get("telemetry") or {}
    truth = snapshot.get("truth") or {}
    vision_tx = snapshot.get("vision_tx") or {}
    frame = snapshot.get("frame") or {}
    pred = match.get("pred_ned") or {}
    seed = telemetry.get("seed_ned") or {}
    return {
        "uptime_sec": snapshot.get("uptime_sec"),
        "video_status": snapshot.get("video_status"),
        "frame_id": frame.get("id"),
        "frame_age_sec": frame.get("age_sec"),
        "match_status": match.get("status"),
        "reject_reason": (match.get("reject") or {}).get("reason") or match.get("reject_reason"),
        "pred_north_m": pred.get("north"),
        "pred_east_m": pred.get("east"),
        "truth_north_m": truth.get("north") if truth.get("fresh") else seed.get("north"),
        "truth_east_m": truth.get("east") if truth.get("fresh") else seed.get("east"),
        "error_m": match.get("independent_error_m") or match.get("error_m"),
        "inliers": match.get("inliers"),
        "good_count": match.get("good_count"),
        "duration_sec": match.get("duration_sec"),
        "telemetry_status": telemetry.get("status"),
        "mavlink_last_message": telemetry.get("last_message"),
        "vision_tx_status": vision_tx.get("status"),
        "vision_tx_sent_count": vision_tx.get("sent_count"),
    }


class ModernDashboardThread(threading.Thread):
    def __init__(
        self,
        host: str,
        port: int,
        state: SharedState,
        preview_map: str,
        static_dir: Optional[str] = None,
    ) -> None:
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.state = state
        self.preview_map = preview_map
        self.static_dir = static_dir
        self.server: Optional[uvicorn.Server] = None

    def run(self) -> None:
        app = create_app(self.state, self.preview_map, self.static_dir)
        config = uvicorn.Config(
            app,
            host=self.host,
            port=self.port,
            log_level="warning",
            access_log=False,
        )
        self.server = uvicorn.Server(config)
        self.server.run()

    def stop(self) -> None:
        if self.server is not None:
            self.server.should_exit = True
