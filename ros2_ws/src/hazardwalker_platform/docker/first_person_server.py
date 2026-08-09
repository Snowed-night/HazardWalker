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
from std_msgs.msg import String


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
    main { position: relative; height: calc(100vh - 44px); width: 100vw; padding: 0; background: #000; }
    #stream { position: absolute; width: 1px; height: 1px; opacity: 0; pointer-events: none; }
    #overlay { display: block; width: 100%; height: 100%; background: #000; }
    .status { color: #93c5fd; font-size: 14px; }
    a { color: #93c5fd; text-decoration: none; }
    button { border: 1px solid #475569; border-radius: 4px; background: #334155; color: #e5e7eb; padding: 5px 9px; cursor: pointer; }
  </style>
</head>
<body>
  <header>
    <strong>HazardWalker · 第一人称 RGB 视角</strong>
    <span class="status" id="status">正在等待相机帧…</span>
    <span><button id="assistStart" type="button">辅助对准</button> <button id="assistCancel" type="button">取消辅助</button> <a href="http://127.0.0.1:6081/hazardwalker.html">打开上帝视角</a> <button id="fullscreen" type="button">全屏</button></span>
  </header>
  <main><img id="stream" src="/mjpeg" alt="官方 RealSense RGB 相机画面"><canvas id="overlay"></canvas></main>
  <script>
    const status = document.getElementById('status');
    const stream = document.getElementById('stream');
    const canvas = document.getElementById('overlay');
    const context = canvas.getContext('2d');
    const frameBuffer = document.createElement('canvas');
    const frameContext = frameBuffer.getContext('2d');
    let frameReady = false;
    let overlayState = { perception: {}, control: {}, assist: {}, state_age_sec: {} };
    const actionLabels = {
      continue_exploring: '继续巡检', turn_left: '左转复查', turn_right: '右转复查',
      move_left: '向左横移', move_right: '向右横移', move_forward: '靠近目标',
      hold_observation: '停稳观察', unknown: '无'
    };
    const assistReasonLabels = {
      waiting_for_user_confirmation: '等待人工确认', assist_not_running: '未运行',
      waiting_for_control_takeover: '等待控制权接管',
      control_takeover_confirmed: '控制权已接管',
      control_takeover_timeout: '控制权接管超时',
      target_centered: '目标已居中', target_not_visible: '目标已离开画面',
      missing_image_width: '缺少画面尺寸', missing_detections: '无候选目标',
      missing_bbox: '候选框无效', invalid_bbox: '候选框无效',
      perception_timeout: '感知超时', alignment_timeout: '对准超时',
      cancelled_by_user: '用户取消', turn_left: '正在左转', turn_right: '正在右转'
    };
    function formatPosition(value) {
      if (!Array.isArray(value) || value.length < 3) return '';
      const numbers = value.slice(0, 3).map(Number);
      if (!numbers.every(Number.isFinite)) return '';
      return `[${numbers.map(number => number.toFixed(2)).join(', ')}]m`;
    }
    document.getElementById('fullscreen').addEventListener('click', async () => {
      if (document.fullscreenElement) await document.exitFullscreen();
      else await document.documentElement.requestFullscreen();
    });
    async function requestAssist(action) {
      if (action === 'start' && !window.confirm('确认让机器狗原地转向并对准当前候选红球？')) return;
      try {
        const response = await fetch(`/assist/${action}`, {
          method: 'POST', cache: 'no-store',
          headers: { 'X-HazardWalker-Confirm': '1' }
        });
        const result = await response.json();
        status.textContent = result.message || (response.ok ? '辅助请求已发送' : '辅助请求失败');
      } catch (_) {
        status.textContent = '辅助请求发送失败';
      }
    }
    document.getElementById('assistStart').addEventListener('click', () => requestAssist('start'));
    document.getElementById('assistCancel').addEventListener('click', () => requestAssist('cancel'));
    stream.addEventListener('load', () => { status.textContent = '相机流连接正常'; });
    stream.addEventListener('error', () => { status.textContent = '相机流断开，正在重连…'; });
    function resizeCanvas() {
      const ratio = Math.min(2, window.devicePixelRatio || 1);
      canvas.width = Math.max(1, Math.round(canvas.clientWidth * ratio));
      canvas.height = Math.max(1, Math.round(canvas.clientHeight * ratio));
    }
    window.addEventListener('resize', resizeCanvas);
    resizeCanvas();
    async function refreshState() {
      try {
        const response = await fetch('/state', { cache: 'no-store' });
        overlayState = await response.json();
      } catch (_) {
        status.textContent = '状态流暂时不可用';
      }
    }
    setInterval(refreshState, 200);
    refreshState();
    function render() {
      const width = canvas.width;
      const height = canvas.height;
      context.clearRect(0, 0, width, height);
      context.fillStyle = '#000';
      context.fillRect(0, 0, width, height);
      const sourceWidth = stream.naturalWidth || overlayState.perception.image_width || 0;
      const sourceHeight = stream.naturalHeight || overlayState.perception.image_height || 0;
      if (sourceWidth > 0 && sourceHeight > 0) {
        if (frameBuffer.width !== sourceWidth || frameBuffer.height !== sourceHeight) {
          frameBuffer.width = sourceWidth;
          frameBuffer.height = sourceHeight;
          frameReady = false;
        }
        // MJPEG 切换帧时 drawImage 偶尔暂时不可用。先更新离屏缓存，失败时
        // 继续使用上一张完整帧，避免主画布因清空后无图而周期性黑闪。
        try {
          frameContext.drawImage(stream, 0, 0, sourceWidth, sourceHeight);
          frameReady = true;
        } catch (_) {}
        const scale = Math.min(width / sourceWidth, height / sourceHeight);
        const drawWidth = sourceWidth * scale;
        const drawHeight = sourceHeight * scale;
        const offsetX = (width - drawWidth) * 0.5;
        const offsetY = (height - drawHeight) * 0.5;
        if (frameReady) {
          context.drawImage(frameBuffer, offsetX, offsetY, drawWidth, drawHeight);
        }
        const age = overlayState.state_age_sec || {};
        const perceptionFresh = age.perception != null && Number(age.perception) <= 1.0;
        const frameStamp = Number((overlayState.frame || {}).ros_stamp_sec);
        const detectionStamp = Number((overlayState.perception || {}).stamp_sec);
        const perceptionSynchronized = Number.isFinite(frameStamp) && Number.isFinite(detectionStamp) && Math.abs(frameStamp - detectionStamp) <= 0.25;
        const perception = perceptionFresh && perceptionSynchronized ? (overlayState.perception || {}) : {};
        const detections = perception.detections_2d || [];
        const hazardsByTrack = new Map(
          (perception.hazards || []).map(hazard => [String(hazard.track_id ?? hazard.id ?? ''), hazard])
        );
        context.lineWidth = Math.max(2, 2 * (window.devicePixelRatio || 1));
        context.font = `${Math.max(14, 14 * (window.devicePixelRatio || 1))}px sans-serif`;
        for (const detection of detections) {
          const box = detection.bbox || {};
          const x = offsetX + Number(box.x_min || 0) * scale;
          const y = offsetY + Number(box.y_min || 0) * scale;
          const w = (Number(box.x_max || 0) - Number(box.x_min || 0)) * scale;
          const h = (Number(box.y_max || 0) - Number(box.y_min || 0)) * scale;
          const trackStatus = String(detection.track_status || '');
          const confirmed = trackStatus === 'confirmed';
          const rejected = trackStatus.startsWith('rejected');
          const color = rejected ? '#a855f7' : (confirmed ? '#22c55e' : (detection.requires_reobservation ? '#f59e0b' : '#ef4444'));
          context.strokeStyle = color;
          context.fillStyle = color;
          context.strokeRect(x, y, w, h);
          const label = rejected ? '已排除非球体' : (confirmed ? '已确认红球' : (detection.requires_reobservation ? '需复查' : '红球候选'));
          const trackId = String(detection.track_id ?? '');
          const candidateId = String(detection.candidate_id ?? '');
          const identity = trackId ? ` #${trackId}` : (candidateId ? ` ${candidateId}` : '');
          context.fillText(`${label}${identity} ${Number(detection.confidence || 0).toFixed(2)}`, x, Math.max(18, y - 6));
          const linkedHazard = hazardsByTrack.get(trackId) || {};
          const position = formatPosition(linkedHazard.position || detection.localized_position);
          const depth = Number(detection.raw_surface_depth_m);
          const depthText = Number.isFinite(depth) && depth > 0 ? `深度 ${depth.toFixed(2)}m` : '';
          const coordinateText = position ? `坐标 ${position}` : '';
          const geometryText = [depthText, coordinateText].filter(Boolean).join(' | ');
          if (geometryText) {
            context.fillText(geometryText, x, Math.min(height - 4, y + h + 18));
          }
        }
      }
      const ages = overlayState.state_age_sec || {};
      const perceptionFresh = ages.perception != null && Number(ages.perception) <= 1.0;
      const controlFresh = ages.control != null && Number(ages.control) <= 2.5;
      const assistFresh = ages.assist != null && Number(ages.assist) <= 2.5;
      const frameStamp = Number((overlayState.frame || {}).ros_stamp_sec);
      const detectionStamp = Number((overlayState.perception || {}).stamp_sec);
      const perceptionSynchronized = Number.isFinite(frameStamp) && Number.isFinite(detectionStamp) && Math.abs(frameStamp - detectionStamp) <= 0.25;
      const perception = perceptionFresh && perceptionSynchronized ? (overlayState.perception || {}) : {};
      const recommendation = perception.view_recommendation || {};
      const control = controlFresh ? (overlayState.control || {}) : {};
      const assist = assistFresh ? (overlayState.assist || {}) : {};
      const perceptionFallback = !perceptionFresh ? '感知状态超时' : (!perceptionSynchronized ? '画面与检测未同步' : '无');
      const action = actionLabels[recommendation.action] || recommendation.action || perceptionFallback;
      const assistState = assist.state || (assistFresh ? '空闲' : '状态超时');
      const assistReason = assistReasonLabels[assist.reason] || assist.reason || '';
      status.textContent = `建议: ${action} | 控制: ${control.mode || (controlFresh ? '未知' : '状态超时')} | 辅助: ${assistState}${assistReason ? `（${assistReason}）` : ''}`;
      setTimeout(render, 50);
    }
    render();
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
        self._ros_stamp_sec: Optional[float] = None

    def update(self, message: CompressedImage) -> None:
        if not message.data:
            return
        with self._condition:
            self._jpeg = bytes(message.data)
            self._sequence += 1
            self._received_at = time.monotonic()
            try:
                self._ros_stamp_sec = float(message.header.stamp.to_sec())
            except (AttributeError, TypeError, ValueError):
                self._ros_stamp_sec = None
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
                "ros_stamp_sec": self._ros_stamp_sec,
                "frame_age_sec": (
                    round(time.monotonic() - self._received_at, 3)
                    if self._received_at else None
                ),
            }


class OverlayState:
    """缓存来自 ROS2 转发的感知、控制和辅助对准 JSON。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._values = {"perception": {}, "control": {}, "assist": {}}
        self._received_at = {"perception": 0.0, "control": 0.0, "assist": 0.0}

    def update(self, name: str, message: String) -> None:
        try:
            value = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        if not isinstance(value, dict):
            return
        with self._lock:
            self._values[name] = value
            self._received_at[name] = time.monotonic()

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            now = time.monotonic()
            snapshot = {
                name: dict(value) for name, value in self._values.items()
            }
            snapshot['state_age_sec'] = {
                name: round(now - received_at, 3) if received_at else None
                for name, received_at in self._received_at.items()
            }
            return snapshot


class FirstPersonRequestHandler(BaseHTTPRequestHandler):
    """暴露只读画面及受限辅助请求，不提供任意 ROS 或速度接口。"""

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
            health = {
                'frame': self.server.frames.state(),
                'overlay': self.server.overlays.snapshot(),
            }
            self._send_bytes(
                json.dumps(health, ensure_ascii=False).encode("utf-8"),
                "application/json; charset=utf-8",
            )
            return
        if path == "/state":
            state = self.server.overlays.snapshot()
            state['frame'] = self.server.frames.state()
            self._send_bytes(
                json.dumps(state, ensure_ascii=False).encode("utf-8"),
                "application/json; charset=utf-8",
            )
            return
        if path == "/mjpeg":
            self._stream_mjpeg()
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        """只接受用户确认的辅助开始/取消，不接收速度或任意话题。"""

        path = urlparse(self.path).path
        actions = {'/assist/start': 'start', '/assist/cancel': 'cancel'}
        action = actions.get(path)
        if action is None:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        # 自定义同源确认头会使跨站脚本先触发 CORS 预检；本服务不开放
        # OPTIONS/CORS，从而避免其他网页在后台向本机控制隧道伪造请求。
        if self.headers.get('X-HazardWalker-Confirm') != '1':
            self._send_bytes(
                json.dumps({
                    'accepted': False,
                    'message': '缺少本地人工确认标记',
                }, ensure_ascii=False).encode('utf-8'),
                'application/json; charset=utf-8',
                status=HTTPStatus.FORBIDDEN,
            )
            return
        self.server.assist_request_pub.publish(String(data=action))
        self._send_bytes(
            json.dumps({
                'accepted': True,
                'action': action,
                'message': '辅助请求已发送，等待控制状态更新',
            }, ensure_ascii=False).encode('utf-8'),
            'application/json; charset=utf-8',
            status=HTTPStatus.ACCEPTED,
        )

    def _send_bytes(
        self, payload: bytes, content_type: str,
        status: HTTPStatus = HTTPStatus.OK,
    ) -> None:
        self.send_response(status)
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

    def __init__(
        self, address, handler, frames: LatestFrame,
        overlays: OverlayState, max_fps: float, assist_request_pub,
    ) -> None:
        super().__init__(address, handler)
        self.frames = frames
        self.overlays = overlays
        self.max_fps = max_fps
        self.assist_request_pub = assist_request_pub


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
    overlays = OverlayState()
    assist_request_pub = rospy.Publisher(
        '/hazardwalker/gui/assist_request', String, queue_size=1)
    rospy.Subscriber(args.topic, CompressedImage, frames.update, queue_size=1, buff_size=2**24)
    rospy.Subscriber(
        '/hazardwalker/gui/perception', String,
        lambda message: overlays.update('perception', message), queue_size=1)
    rospy.Subscriber(
        '/hazardwalker/gui/control_status', String,
        lambda message: overlays.update('control', message), queue_size=1)
    rospy.Subscriber(
        '/hazardwalker/gui/assist_status', String,
        lambda message: overlays.update('assist', message), queue_size=1)
    # 页面直接输出官方压缩话题，不进行第二次 JPEG 编码。提高原话题质量可消除
    # 浏览器放大后明显的压缩块；该参数只影响有人订阅时的编码负载。
    rospy.set_param(f"{args.topic}/jpeg_quality", args.jpeg_quality)
    rospy.set_param(f"{args.topic}/jpeg_progressive", False)
    rospy.set_param(f"{args.topic}/jpeg_optimize", True)
    server = FirstPersonHttpServer(
        (args.host, args.port), FirstPersonRequestHandler,
        frames, overlays, args.max_fps, assist_request_pub)
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
