"""USB カメラの取得と MJPEG 配信。"""
from __future__ import annotations

import asyncio
import logging
import threading
import time
from typing import Any, AsyncIterator, Callable

import cv2
import numpy as np

from .config import CameraConfig

log = logging.getLogger(__name__)

BOUNDARY = "frame"
_PART_HEADER = f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\n\r\n".encode()


class Camera:
    def __init__(
        self,
        cfg: CameraConfig,
        mock: bool = False,
        opener: Callable[[], Any] | None = None,
        retry_s: float = 3.0,
    ) -> None:
        self._cfg = cfg
        self._mock = mock
        self._opener = opener or self._default_opener
        self._retry_s = retry_s
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._seq = 0
        self.ok = False
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def _default_opener(self) -> Any:
        cap = cv2.VideoCapture(self._cfg.device)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._cfg.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._cfg.height)
        cap.set(cv2.CAP_PROP_FPS, self._cfg.fps)
        return cap

    def start(self) -> None:
        target = self._run_mock if self._mock else self._run
        self._thread = threading.Thread(target=target, name="camera", daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=5)

    def latest_jpeg(self) -> bytes | None:
        with self._lock:
            return self._jpeg

    async def mjpeg(self) -> AsyncIterator[bytes]:
        last_seq = -1
        interval = 1.0 / self._cfg.fps
        while not self._stop_event.is_set():
            with self._lock:
                jpeg, seq = self._jpeg, self._seq
            if jpeg is not None and seq != last_seq:
                last_seq = seq
                yield _PART_HEADER + jpeg + b"\r\n"
            await asyncio.sleep(interval)

    def _publish(self, frame: np.ndarray) -> None:
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._cfg.jpeg_quality])
        if not ok:
            return
        with self._lock:
            self._jpeg = buf.tobytes()
            self._seq += 1
        self.ok = True

    def _set_down(self) -> None:
        self.ok = False
        with self._lock:
            self._jpeg = None

    def _run(self) -> None:
        while not self._stop_event.is_set():
            cap = self._opener()
            if not cap.isOpened():
                log.warning("カメラ %s を開けません。%.0f秒後に再試行します", self._cfg.device, self._retry_s)
            else:
                while not self._stop_event.is_set():
                    ok, frame = cap.read()
                    if not ok:
                        log.warning("カメラの読み取りに失敗しました。再接続します")
                        break
                    self._publish(frame)
            cap.release()
            self._set_down()
            self._stop_event.wait(self._retry_s)

    def _run_mock(self) -> None:
        interval = 1.0 / self._cfg.fps
        while not self._stop_event.is_set():
            frame = np.full((self._cfg.height, self._cfg.width, 3), 80, dtype=np.uint8)
            text = time.strftime("MOCK %H:%M:%S")
            cv2.putText(frame, text, (10, self._cfg.height // 2), cv2.FONT_HERSHEY_SIMPLEX,
                        self._cfg.width / 640, (255, 255, 255), 2)
            self._publish(frame)
            self._stop_event.wait(interval)
