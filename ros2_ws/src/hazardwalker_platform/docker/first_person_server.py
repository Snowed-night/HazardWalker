#!/usr/bin/env python3
"""HazardWalker 第一人称浏览器视频服务。

订阅官方 ROS1 压缩 RGB 话题，仅缓存最新 JPEG 帧并通过本地 MJPEG 页面输出。
该服务不启动 Gazebo、不读取真值、不发布控制指令，可与 6081 上帝视角并行运行。
"""

from __future__ import annotations

import argparse
import json
import signal
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional, Tuple
from urllib.parse import urlparse

import rospy
from sensor_msgs.msg import CompressedImage


PAGE_HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>HazardWalker 第一人称视角</title>
  <style>
    * { box-sizing: border-box; }
    html, body { width: 100%; height: 100%; overflow: hidden; }
    body { margin: 0; background: #111827; color: #e5e7eb; font-family: system-ui, sans-serif; }
    header { height: 44px; display: flex; align-items: center; justify-content: space-between; gap: 16px; padding: 0 18px; background: #1f2937; }
    main { height: calc(100vh - 44px); width: 100vw; display: grid; place-items: center; padding: 0; }
    #stream { display: block; width: 100%; height: 100%; object-fit: contain; background: #000; image-rendering: auto; }
    .status { color: #93c5fd; font-size: 14px; }
    a { color: #93c5fd; text-decoration: none; }
    button { border: 1px solid #475569; border-radius: 4px; background: #334155; color: #e5e7eb; padding: 5px 9px; cursor: pointer; }
  </style>
</head>
<body>
  <header>
    <strong>HazardWalker · 第一人称 RGB 视角</strong>
    <span class="status" id="status">正在等待相机帧…</span>
    <span><a href="http://127.0.0.1:6081/hazardwalker.html">打开上帝视角</a> <button id="fullscreen" type="button">全屏</button></span>
  </header>
  <main><img id="stream" src="/mjpeg" alt="官方 RealSense RGB 相机画面"></main>
  <script>
    const status = document.getElementById('status');
    const stream = document.getElementById('stream');
    document.getElementById('fullscreen').addEventListener('click', async () => {
      if (document.fullscreenElement) await document.exitFullscreen();
      else await document.documentElement.requestFullscreen();
    });
    stream.addEventListener('load', () => { status.textContent = '相机流连接正常'; });
    stream.addEventListener('error', () => { status.textContent = '相机流断开，正在重连…'; });
  </script>
</body>
</html>
"""


class LatestFrame:
    """线程安全的最新 JPEG 帧缓存；慢客户端不会积压历史画面。"""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._jpeg: Optional[bytes] = None
        self._sequence = 0
        self._received_at = 0.0

    def update(self, message: CompressedImage) -> None:
        if not message.data:
            return
        with self._condition:
            self._jpeg = bytes(message.data)
            self._sequence += 1
            self._received_at = time.monotonic()
            self._condition.notify_all()

    def wait_for_newer(
        self, sequence: int, timeout: float
    ) -> Tuple[Optional[bytes], int]:
        """等待更新；超时后仍返回最近一帧，避免浏览器误判服务断开。"""

        with self._condition:
            if self._sequence <= sequence:
                self._condition.wait(timeout=timeout)
            return self._jpeg, self._sequence

    def state(self) -> dict[str, object]:
        with self._condition:
            return {
                "ready": self._jpeg is not None,
                "frame_sequence": self._sequence,
                "frame_age_sec": (
                    round(time.monotonic() - self._received_at, 3)
                    if self._received_at else None
                ),
            }


class FirstPersonRequestHandler(BaseHTTPRequestHandler):
    """只暴露页面、MJPEG 和健康检查，避免将 ROS 接口暴露到浏览器。"""

    server: "FirstPersonHttpServer"

    def log_message(self, _format: str, *_args: object) -> None:
        """HTTP 客户端刷新频繁，日志仅保留 ROS 侧关键状态。"""

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        path = urlparse(self.path).path
        if path == "/":
            self.send_response(HTTPStatus.FOUND)
            self.send_header("Location", "/first_person")
            self.end_headers()
            return
        if path == "/first_person":
            self._send_bytes(PAGE_HTML.encode("utf-8"), "text/html; charset=utf-8")
            return
        if path == "/healthz":
            self._send_bytes(
                json.dumps(self.server.frames.state()).encode("utf-8"),
                "application/json; charset=utf-8",
            )
            return
        if path == "/mjpeg":
            self._stream_mjpeg()
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def _send_bytes(self, payload: bytes, content_type: str) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _stream_mjpeg(self) -> None:
        boundary = b"hazardwalker-frame"
        self.send_response(HTTPStatus.OK)
        self.send_header(
            "Content-Type", f"multipart/x-mixed-replace; boundary={boundary.decode()}"
        )
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.end_headers()

        sequence = -1
        min_interval = 1.0 / self.server.max_fps
        try:
            while not rospy.is_shutdown():
                started = time.monotonic()
                jpeg, sequence = self.server.frames.wait_for_newer(sequence, timeout=2.0)
                if jpeg is None:
                    continue
                self.wfile.write(b"--" + boundary + b"\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                self.wfile.write(jpeg + b"\r\n")
                self.wfile.flush()
                remaining = min_interval - (time.monotonic() - started)
                if remaining > 0:
                    time.sleep(remaining)
        except (BrokenPipeError, ConnectionResetError):
            return


class FirstPersonHttpServer(ThreadingHTTPServer):
    """将 ROS 帧缓存和输出帧率传给请求处理器。"""

    daemon_threads = True

    def __init__(self, address, handler, frames: LatestFrame, max_fps: float) -> None:
        super().__init__(address, handler)
        self.frames = frames
        self.max_fps = max_fps


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="HazardWalker first-person MJPEG server")
    parser.add_argument("--topic", default="/real_sense/rgb/image_raw/compressed")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=6082)
    parser.add_argument("--max-fps", type=float, default=15.0)
    parser.add_argument("--jpeg-quality", type=int, default=92)
    return parser.parse_args()


def main() -> int:
    """启动相机订阅和本地 HTTP 服务，信号退出时不影响主仿真。"""

    args = parse_args()
    if args.port <= 0 or args.port > 65535 or args.max_fps <= 0:
        raise ValueError("port 必须为 1-65535，max-fps 必须为正数")
    if args.jpeg_quality < 1 or args.jpeg_quality > 100:
        raise ValueError("jpeg-quality 必须为 1-100")

    rospy.init_node("hazardwalker_first_person_web", disable_signals=True)
    frames = LatestFrame()
    rospy.Subscriber(args.topic, CompressedImage, frames.update, queue_size=1, buff_size=2**24)
    # 页面直接输出官方压缩话题，不进行第二次 JPEG 编码。提高原话题质量可消除
    # 浏览器放大后明显的压缩块；该参数只影响有人订阅时的编码负载。
    rospy.set_param(f"{args.topic}/jpeg_quality", args.jpeg_quality)
    rospy.set_param(f"{args.topic}/jpeg_progressive", False)
    rospy.set_param(f"{args.topic}/jpeg_optimize", True)
    server = FirstPersonHttpServer((args.host, args.port), FirstPersonRequestHandler, frames, args.max_fps)
    rospy.loginfo(
        "第一人称视频服务已启动：topic=%s, url=http://%s:%s/first_person, max_fps=%.1f, jpeg_quality=%d",
        args.topic,
        args.host,
        args.port,
        args.max_fps,
        args.jpeg_quality,
    )

    def stop_server(_signum, _frame) -> None:
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGINT, stop_server)
    signal.signal(signal.SIGTERM, stop_server)
    try:
        server.serve_forever(poll_interval=0.5)
    finally:
        server.server_close()
        rospy.signal_shutdown("first-person web server stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
