#!/usr/bin/env python3
"""Capture Gazebo red-ball detection examples into one report directory."""

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

from PIL import Image, ImageDraw


REPO_ROOT = Path(__file__).resolve().parents[1]
PLATFORM_SRC = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_platform'
DEFAULT_WORLD = (
    REPO_ROOT
    / 'ros2_ws'
    / 'install'
    / 'hazardwalker_platform'
    / 'share'
    / 'hazardwalker_platform'
    / 'worlds'
    / 'hazardwalker_red_ball_gallery.sdf'
)
DEFAULT_MODELS_DIR = (
    REPO_ROOT
    / 'ros2_ws'
    / 'install'
    / 'hazardwalker_platform'
    / 'share'
    / 'hazardwalker_platform'
    / 'models'
)

CAMERA_CASES = [
    {
        'name': 'center_full',
        'topic': '/gallery/center_full/image',
        'description': 'Single red ball centered and fully visible.',
    },
    {
        'name': 'edge_partial',
        'topic': '/gallery/left_partial/image',
        'description': 'Single red ball clipped by an image edge; useful for partial-visibility checks.',
    },
    {
        'name': 'top_partial',
        'topic': '/gallery/top_partial/image',
        'description': 'Single red ball clipped by the top edge.',
    },
    {
        'name': 'multi_visible',
        'topic': '/gallery/multi_visible/image',
        'description': 'Multiple red balls in one frame.',
    },
]


def _import_detector():
    perception_src = REPO_ROOT / 'ros2_ws' / 'src' / 'hazardwalker_perception'
    sys.path.insert(0, str(perception_src))
    from hazardwalker_perception.red_ball_detector import detect_red_ball_rgb_bytes

    return detect_red_ball_rgb_bytes


def _read_gz_image_text(topic, partition, timeout_sec):
    env = os.environ.copy()
    env['GZ_PARTITION'] = partition
    result = subprocess.run(
        ['gz', 'topic', '-e', '-t', topic, '-n', '1'],
        check=False,
        capture_output=True,
        text=False,
        timeout=timeout_sec,
        env=env,
    )
    if result.returncode != 0:
        stderr = result.stderr.decode(errors='replace')
        raise RuntimeError(f'gz topic failed for {topic}: {stderr.strip()}')
    return result.stdout.decode('latin1')


def _extract_field_int(text, field):
    marker = f'{field}:'
    start = text.find(marker)
    if start < 0:
        raise RuntimeError(f'missing field {field}')
    start += len(marker)
    end = text.find('\n', start)
    return int(text[start:end].strip())


def _extract_data_bytes(text, expected_len):
    marker = 'data: "'
    start = text.find(marker)
    if start < 0:
        raise RuntimeError('missing image data field')
    start += len(marker)
    chars = []
    escaped = False
    for ch in text[start:]:
        if escaped:
            chars.append('\\' + ch)
            escaped = False
            continue
        if ch == '\\':
            escaped = True
            continue
        if ch == '"':
            break
        chars.append(ch)
    escaped_text = ''.join(chars)
    raw = escaped_text.encode('latin1').decode('unicode_escape').encode('latin1')
    if len(raw) < expected_len:
        raise RuntimeError(f'image data too short: {len(raw)} < {expected_len}')
    return raw[:expected_len]


def _extract_pixel_format(text):
    marker = 'pixel_format_type:'
    start = text.find(marker)
    if start < 0:
        return 'UNKNOWN'
    start += len(marker)
    end = text.find('\n', start)
    return text[start:end].strip()


def _gz_text_to_rgb_image(text):
    width = _extract_field_int(text, 'width')
    height = _extract_field_int(text, 'height')
    pixel_format = _extract_pixel_format(text)
    raw = _extract_data_bytes(text, width * height * 3)
    # Gallery cameras publish RGB_INT8 in Gazebo Harmonic.
    image = Image.frombytes('RGB', (width, height), raw)
    return image, pixel_format, raw


def _draw_detection(image, detection):
    boxed = image.copy()
    draw = ImageDraw.Draw(boxed)
    if detection is None:
        draw.rectangle([8, 8, 180, 32], fill=(255, 255, 255))
        draw.text((12, 13), 'no red ball detected', fill=(180, 0, 0))
        return boxed

    x0 = detection.x_min
    y0 = detection.y_min
    x1 = detection.x_max
    y1 = detection.y_max
    for pad in range(4):
        draw.rectangle([x0 - pad, y0 - pad, x1 + pad, y1 + pad], outline=(255, 0, 0))

    label = f'red pixels={detection.red_pixel_count} conf={detection.confidence:.2f}'
    label_y0 = max(0, y0 - 20)
    draw.rectangle([max(0, x0), label_y0, min(image.width - 1, x0 + 250), label_y0 + 18],
                   fill=(255, 255, 255))
    draw.text((max(0, x0 + 3), label_y0 + 3), label, fill=(255, 0, 0))
    return boxed


def _find_red_components(raw, width, height, min_area_px=20):
    # Reuse the same HSV thresholds through the public detector helpers.
    from hazardwalker_perception.red_ball_detector import is_red_hsv, rgb_to_hsv_pixel

    red = bytearray(width * height)
    for y in range(height):
        row = y * width * 3
        mask_row = y * width
        for x in range(width):
            index = row + x * 3
            r, g, b = raw[index], raw[index + 1], raw[index + 2]
            h, s, v = rgb_to_hsv_pixel(r, g, b)
            if is_red_hsv(h, s, v):
                red[mask_row + x] = 1

    visited = bytearray(width * height)
    components = []
    for start in range(width * height):
        if not red[start] or visited[start]:
            continue
        stack = [start]
        visited[start] = 1
        count = 0
        x_min = width
        y_min = height
        x_max = 0
        y_max = 0
        while stack:
            idx = stack.pop()
            x = idx % width
            y = idx // width
            count += 1
            x_min = min(x_min, x)
            y_min = min(y_min, y)
            x_max = max(x_max, x)
            y_max = max(y_max, y)
            for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if nx < 0 or nx >= width or ny < 0 or ny >= height:
                    continue
                nidx = ny * width + nx
                if red[nidx] and not visited[nidx]:
                    visited[nidx] = 1
                    stack.append(nidx)
        if count >= min_area_px:
            components.append({
                'x_min': x_min,
                'y_min': y_min,
                'x_max': x_max,
                'y_max': y_max,
                'red_pixel_count': count,
            })

    components.sort(key=lambda item: item['red_pixel_count'], reverse=True)
    return components


def _draw_components(image, components):
    boxed = image.copy()
    draw = ImageDraw.Draw(boxed)
    if not components:
        draw.rectangle([8, 8, 180, 32], fill=(255, 255, 255))
        draw.text((12, 13), 'no red ball detected', fill=(180, 0, 0))
        return boxed

    for index, comp in enumerate(components, start=1):
        x0 = comp['x_min']
        y0 = comp['y_min']
        x1 = comp['x_max']
        y1 = comp['y_max']
        for pad in range(3):
            draw.rectangle([x0 - pad, y0 - pad, x1 + pad, y1 + pad], outline=(255, 0, 0))
        label = f'#{index} red={comp["red_pixel_count"]}'
        label_y0 = max(0, y0 - 18)
        draw.rectangle([max(0, x0), label_y0, min(image.width - 1, x0 + 120), label_y0 + 16],
                       fill=(255, 255, 255))
        draw.text((max(0, x0 + 3), label_y0 + 2), label, fill=(255, 0, 0))
    return boxed


def _start_gazebo(world, models_dir, partition):
    env = os.environ.copy()
    env['GZ_PARTITION'] = partition
    env['GZ_SIM_RESOURCE_PATH'] = str(models_dir)
    return subprocess.Popen(
        ['gz', 'sim', '-r', '-s', '--headless-rendering', '-v', '2', str(world)],
        cwd=str(REPO_ROOT),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
    )


def _stop_process(proc):
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=3)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', type=Path, default=DEFAULT_WORLD)
    parser.add_argument('--models-dir', type=Path, default=DEFAULT_MODELS_DIR)
    parser.add_argument('--output-dir', type=Path, default=None)
    parser.add_argument('--startup-wait', type=float, default=3.0)
    parser.add_argument('--timeout', type=float, default=10.0)
    args = parser.parse_args()

    if not args.world.is_file():
        raise SystemExit(f'world not found: {args.world}\nRun ./scripts/build.sh first.')
    if not args.models_dir.is_dir():
        raise SystemExit(f'models dir not found: {args.models_dir}\nRun ./scripts/build.sh first.')

    detect_red_ball_rgb_bytes = _import_detector()
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = args.output_dir or (REPO_ROOT / 'reports' / 'red_ball_gallery' / timestamp)
    output_dir.mkdir(parents=True, exist_ok=True)

    partition = f'hw_gallery_{timestamp}_{os.getpid()}'
    proc = _start_gazebo(args.world, args.models_dir, partition)
    summary = {
        'created_at': timestamp,
        'world': str(args.world),
        'partition': partition,
        'cases': [],
    }

    try:
        time.sleep(args.startup_wait)
        for case in CAMERA_CASES:
            text = _read_gz_image_text(case['topic'], partition, args.timeout)
            image, pixel_format, raw = _gz_text_to_rgb_image(text)
            detection = detect_red_ball_rgb_bytes(
                raw,
                image.width,
                image.height,
                step=image.width * 3,
                encoding='rgb8',
                min_area_px=20,
            )
            components = _find_red_components(raw, image.width, image.height, min_area_px=20)

            raw_path = output_dir / f'{case["name"]}_raw.png'
            boxed_path = output_dir / f'{case["name"]}_boxed.png'
            image.save(raw_path)
            _draw_components(image, components).save(boxed_path)

            detection_data = None
            if detection is not None:
                detection_data = {
                    'x_min': detection.x_min,
                    'y_min': detection.y_min,
                    'x_max': detection.x_max,
                    'y_max': detection.y_max,
                    'confidence': detection.confidence,
                    'red_pixel_count': detection.red_pixel_count,
                }

            summary['cases'].append({
                'name': case['name'],
                'topic': case['topic'],
                'description': case['description'],
                'pixel_format': pixel_format,
                'image_size': [image.width, image.height],
                'raw_image': str(raw_path.relative_to(REPO_ROOT)),
                'boxed_image': str(boxed_path.relative_to(REPO_ROOT)),
                'detection': detection_data,
                'components': components,
            })

        summary_path = output_dir / 'summary.json'
        summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
                                encoding='utf-8')
        print(f'Output directory: {output_dir}')
        print(f'Summary: {summary_path}')
        for case in summary['cases']:
            detection = case['detection']
            components = case['components']
            if detection is None:
                print(f'- {case["name"]}: no detector candidate, red_components={len(components)}')
            else:
                bbox = (
                    detection['x_min'],
                    detection['y_min'],
                    detection['x_max'],
                    detection['y_max'],
                )
                print(
                    f'- {case["name"]}: bbox={bbox} '
                    f'red_pixels={detection["red_pixel_count"]} '
                    f'red_components={len(components)}'
                )
    finally:
        _stop_process(proc)


if __name__ == '__main__':
    main()
