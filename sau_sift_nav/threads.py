from __future__ import annotations

import threading
import time
from datetime import datetime
from typing import Optional

from .localizer import SiftLocalizer
from .logging import append_match_log, append_validation_log
from .state import SharedState


class MatcherThread(threading.Thread):
    def __init__(
        self,
        state: SharedState,
        localizer: SiftLocalizer,
        interval_sec: float,
        log_csv: Optional[str],
        allow_gps_search_seed: bool = True,
    ) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.localizer = localizer
        self.interval_sec = interval_sec
        self.log_csv = log_csv
        self.allow_gps_search_seed = allow_gps_search_seed
        self.stop_event = threading.Event()
        self.last_frame_id = -1
        self.last_started = 0.0

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        while not self.stop_event.is_set():
            if time.time() - self.last_started < self.interval_sec:
                time.sleep(0.05)
                continue

            frame_id, frame, frame_time = self.state.get_frame()
            if frame is None or frame_id == self.last_frame_id:
                time.sleep(0.05)
                continue

            self.last_started = time.time()
            self.last_frame_id = frame_id
            self.state.set_matcher_busy(True)
            try:
                hint = self.state.search_hint(allow_gps_seed=self.allow_gps_search_seed)
                result = self.localizer.match_frame(frame, search_hint=hint)
                result["frame_id"] = frame_id
                result["frame_capture_time"] = frame_time
                result["timestamp"] = datetime.now().isoformat(timespec="seconds")
                self.state.update_match(result)
                if self.log_csv:
                    append_match_log(self.log_csv, result, self.state.snapshot())
            except Exception as exc:
                self.state.set_error(f"Matcher error: {exc}")
            finally:
                self.state.set_matcher_busy(False)


class ValidationLoggerThread(threading.Thread):
    def __init__(self, state: SharedState, log_csv: str, rate_hz: float = 10.0) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.log_csv = log_csv
        self.rate_hz = max(0.1, float(rate_hz))
        self.stop_event = threading.Event()

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        period = 1.0 / self.rate_hz
        while not self.stop_event.is_set():
            started = time.time()
            try:
                append_validation_log(self.log_csv, self.state.snapshot(), started)
            except Exception as exc:
                self.state.set_error(f"Validation log failed: {exc}")
            elapsed = time.time() - started
            time.sleep(max(0.0, period - elapsed))
