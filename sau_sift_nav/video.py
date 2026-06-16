from __future__ import annotations

import threading
import time

import cv2

from .state import SharedState


def make_gst_pipeline(port: int) -> str:
    return (
        f"udpsrc port={port} "
        "caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, "
        "encoding-name=(string)H264\" ! "
        "rtph264depay ! avdec_h264 ! videoconvert ! "
        "appsink drop=true sync=false max-buffers=1"
    )


class VideoThread(threading.Thread):
    def __init__(self, state: SharedState, source: str, udp_port: int) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.source = source
        self.udp_port = udp_port
        self.stop_event = threading.Event()

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        cap = self._open_capture()
        if not cap.isOpened():
            self.state.set_video_status("open_failed")
            self.state.set_error("Video source could not be opened")
            return

        self.state.set_video_status("open")
        while not self.stop_event.is_set():
            ok, frame = cap.read()
            if not ok or frame is None:
                self.state.set_video_status("waiting_frame")
                time.sleep(0.08)
                continue
            self.state.update_frame(frame)
            self.state.set_video_status("streaming")

        cap.release()
        self.state.set_video_status("stopped")

    def _open_capture(self) -> cv2.VideoCapture:
        if self.source == "udp":
            return cv2.VideoCapture(make_gst_pipeline(self.udp_port), cv2.CAP_GSTREAMER)
        if self.source.startswith("gst:"):
            return cv2.VideoCapture(self.source[4:], cv2.CAP_GSTREAMER)
        if self.source.isdigit():
            return cv2.VideoCapture(int(self.source))
        return cv2.VideoCapture(self.source)
