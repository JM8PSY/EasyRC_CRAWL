"""FastAPI サーバー: 操縦指令 (/ws)、映像 (/video)、状態 (/status)。"""
from __future__ import annotations

import argparse
import asyncio
import atexit
import json
import logging
import os
import sys
import time
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from .camera import BOUNDARY, Camera
from .config import Config, load_config
from .motor import BackendError, MockBackend, MotorController, PigpioBackend
from .watchdog import Watchdog

log = logging.getLogger(__name__)

STATIC_DIR = Path(__file__).parent / "static"
CONTROL_HZ = 50


def create_app(cfg: Config, motor: MotorController, camera: Camera) -> FastAPI:
    watchdog = Watchdog(cfg.watchdog.timeout_ms / 1000, motor.stop)
    driver: dict[str, WebSocket | None] = {"ws": None}

    async def control_loop() -> None:
        last = time.monotonic()
        while True:
            await asyncio.sleep(1 / CONTROL_HZ)
            now = time.monotonic()
            motor.tick(now - last)
            last = now

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        camera.start()
        watchdog.start()
        task = asyncio.create_task(control_loop())
        try:
            yield
        finally:
            task.cancel()
            watchdog.shutdown()
            motor.close()
            camera.shutdown()

    app = FastAPI(lifespan=lifespan)
    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

    @app.get("/")
    async def index() -> FileResponse:
        return FileResponse(STATIC_DIR / "index.html")

    @app.get("/video")
    async def video() -> StreamingResponse:
        return StreamingResponse(
            camera.mjpeg(), media_type=f"multipart/x-mixed-replace; boundary={BOUNDARY}"
        )

    @app.get("/status")
    async def status() -> dict:
        return {"camera": camera.ok, "driver_connected": driver["ws"] is not None}

    def handle(text: str) -> None:
        try:
            msg = json.loads(text)
            if not isinstance(msg, dict):
                raise ValueError("JSON オブジェクトではありません")
            if msg.get("stop") is True:
                motor.stop()
                return
            motor.set_target(msg["l"], msg["r"])
        except (ValueError, KeyError) as e:
            log.warning("不正な指令を無視しました: %r (%s)", text[:100], e)
            return
        watchdog.feed()

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket) -> None:
        await ws.accept()
        is_driver = driver["ws"] is None
        if is_driver:
            driver["ws"] = ws
            log.info("操縦者が接続しました")
        try:
            await ws.send_json({"role": "driver" if is_driver else "viewer"})
            while True:
                text = await ws.receive_text()
                if is_driver:
                    handle(text)
        except WebSocketDisconnect:
            pass
        finally:
            if is_driver:
                motor.stop()
                driver["ws"] = None
                log.info("操縦者が切断したため停止しました")

    return app


def main(argv: list[str] | None = None) -> None:
    import uvicorn

    parser = argparse.ArgumentParser(description="CuGo V3 Web遠隔操縦サーバー")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--mock", action="store_true", help="GPIO とカメラをダミーにする")
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    cfg = load_config(args.config if os.path.exists(args.config) else None)

    try:
        backend = MockBackend() if args.mock else PigpioBackend()
    except BackendError as e:
        print(e, file=sys.stderr)
        sys.exit(1)
    motor = MotorController(cfg.motor, backend)

    def safe_stop() -> None:
        try:
            motor.stop()
        except Exception:
            pass

    atexit.register(safe_stop)
    camera = Camera(cfg.camera, mock=args.mock)
    uvicorn.run(create_app(cfg, motor, camera), host=cfg.server.host, port=cfg.server.port)


if __name__ == "__main__":
    main()
