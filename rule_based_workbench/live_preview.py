"""Bounded latest-frame preview transport for the data workbench.

The producer writes one atomic file.  The stream process keeps one persistent
reader and emits only the newest frame, so a slow browser never queues stale
frames or opens one SSH connection per image.
"""
from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import tempfile
import time
from dataclasses import dataclass
from pathlib import Path
from typing import BinaryIO

_MAGIC = b"LWPF1\0"
_HEADER = struct.Struct("!QI")  # payload length, metadata length
PREVIEW_CAMERAS = ("base_0", "left_wrist_0", "right_wrist_0")


def camera_frame_path(directory: os.PathLike[str] | str, camera: str = "base_0") -> Path:
    """Head retains its legacy filename; wrists get independent latest-only slots."""
    if camera not in PREVIEW_CAMERAS:
        raise ValueError(f"unknown preview camera: {camera}")
    suffix = "" if camera == "base_0" else "-" + camera
    return Path(directory) / f"live-preview{suffix}.frame"


@dataclass(frozen=True)
class PreviewFrame:
    metadata: dict
    payload: bytes


def _encode(frame: PreviewFrame) -> bytes:
    meta = json.dumps(frame.metadata, ensure_ascii=False, separators=(",", ":"), allow_nan=False).encode()
    return _MAGIC + _HEADER.pack(len(frame.payload), len(meta)) + meta + frame.payload


def _decode(blob: bytes) -> PreviewFrame:
    if len(blob) < len(_MAGIC) + _HEADER.size or blob[:len(_MAGIC)] != _MAGIC:
        raise ValueError("invalid preview frame")
    payload_size, meta_size = _HEADER.unpack_from(blob, len(_MAGIC))
    start = len(_MAGIC) + _HEADER.size
    end_meta = start + meta_size
    end_payload = end_meta + payload_size
    if end_payload != len(blob):
        raise ValueError("truncated preview frame")
    return PreviewFrame(json.loads(blob[start:end_meta]), blob[end_meta:end_payload])


def write_frame(path: os.PathLike[str] | str, payload: bytes, metadata: dict,
                *, durable: bool = True) -> None:
    """Atomically replace the one-slot cache.  Readers see old or new, never partial."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    blob = _encode(PreviewFrame(dict(metadata), bytes(payload)))
    fd, temp_name = tempfile.mkstemp(prefix=target.name + ".", dir=target.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(blob)
            stream.flush()
            if durable:
                os.fsync(stream.fileno())
        os.replace(temp_name, target)
    except BaseException:
        try:
            os.unlink(temp_name)
        except FileNotFoundError:
            pass
        raise


def read_frame(path: os.PathLike[str] | str) -> PreviewFrame | None:
    try:
        return _decode(Path(path).read_bytes())
    except FileNotFoundError:
        return None


def publish_rgb(publisher: "PreviewPublisher", rgb, metadata: dict | None = None) -> dict:
    """Encode an Isaac/PIL-compatible RGB array and publish one latest frame."""
    from io import BytesIO
    from PIL import Image
    if hasattr(rgb, "detach"):
        rgb = rgb.detach().cpu().numpy()
    image = Image.fromarray(rgb[..., :3])
    output = BytesIO()
    image.save(output, format="JPEG", quality=75, optimize=False)
    return publisher.publish(output.getvalue(), metadata)


class PreviewPublisher:
    """Latest-only publisher; callers may publish each simulation control tick."""

    def __init__(self, path: os.PathLike[str] | str, *, durable: bool = True):
        self.path = Path(path)
        self.durable = durable
        self.sequence = 0
        self.published = 0
        self.started_ns = time.time_ns()

    def publish(self, payload: bytes, metadata: dict | None = None) -> dict:
        self.sequence += 1
        now_ns = time.time_ns()
        info = dict(metadata or {})
        info.update(sequence=self.sequence, published_at_ns=now_ns,
                    control_hz=float(info.get("control_hz", 30.0)),
                    source_fps=(self.sequence * 1e9 / max(1, now_ns - self.started_ns)))
        write_frame(self.path, payload, info, durable=self.durable)
        self.published += 1
        return info

    def stats(self) -> dict:
        elapsed = max((time.time_ns() - self.started_ns) / 1e9, 1e-9)
        return {"published_frames": self.published, "elapsed_s": elapsed,
                "source_fps": self.published / elapsed, "path": str(self.path)}


class LatestFrameStream:
    """Persistent stream sender.  Polling intentionally drops intermediate frames."""

    def __init__(self, path: os.PathLike[str] | str, output: BinaryIO = sys.stdout.buffer,
                 hz: float = 30.0):
        if not hz > 0:
            raise ValueError("hz must be positive")
        self.path = Path(path)
        self.output = output
        self.period = 1.0 / hz
        self._last_sequence = 0

    def run(self, stop_after: float | None = None) -> dict:
        started = time.monotonic()
        sent = 0
        while stop_after is None or time.monotonic() - started < stop_after:
            frame = read_frame(self.path)
            if frame is not None and int(frame.metadata.get("sequence", 0)) > self._last_sequence:
                blob = _encode(frame)
                self.output.write(struct.pack("!Q", len(blob)) + blob)
                self.output.flush()
                self._last_sequence = int(frame.metadata["sequence"])
                sent += 1
            if any((self.path.parent/name).is_file() for name in ('failure.json','final.json')):
                break
            time.sleep(self.period)
        elapsed = max(time.monotonic() - started, 1e-9)
        return {"sent_frames": sent, "elapsed_s": elapsed, "stream_fps": sent / elapsed,
                "last_sequence": self._last_sequence}


def receive_stream(stream: BinaryIO, cache_path: os.PathLike[str] | str,
                   stop_after: float | None = None) -> dict:
    """Consume framed stdout and atomically cache latest frame for local HTTP."""
    started = time.monotonic()
    received = 0
    buffer = bytearray()
    while stop_after is None or time.monotonic() - started < stop_after:
        try:
            chunk = stream.read(64 * 1024)
        except (OSError, ValueError):
            break
        if not chunk:
            break
        buffer.extend(chunk)
        while True:
            # SSH shells may print a login banner before binary stdout. The
            # inner magic is authoritative; discard everything before it.
            start = buffer.find(_MAGIC)
            if start < 0:
                buffer[:] = buffer[-len(_MAGIC) + 1:]
                break
            if start:
                del buffer[:start]
            if len(buffer) < len(_MAGIC) + _HEADER.size:
                break
            payload_size, meta_size = _HEADER.unpack_from(buffer, len(_MAGIC))
            frame_size = len(_MAGIC) + _HEADER.size + meta_size + payload_size
            if frame_size > 128 * 1024 * 1024:
                raise ValueError("preview frame exceeds 128 MiB")
            if len(buffer) < frame_size:
                break
            blob = bytes(buffer[:frame_size])
            del buffer[:frame_size]
            frame = _decode(blob)
            write_frame(cache_path, frame.payload, frame.metadata)
            received += 1
    elapsed = max(time.monotonic() - started, 1e-9)
    return {"received_frames": received, "elapsed_s": elapsed,
            "receive_fps": received / elapsed}


def _main() -> None:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    send = sub.add_parser("stream")
    send.add_argument("--path", type=Path, required=True)
    send.add_argument("--hz", type=float, default=30.0)
    send.add_argument("--seconds", type=float)
    recv = sub.add_parser("receive")
    recv.add_argument("--cache", type=Path, required=True)
    recv.add_argument("--seconds", type=float)
    args = parser.parse_args()
    if args.command == "stream":
        print(json.dumps(LatestFrameStream(args.path, hz=args.hz).run(args.seconds)), file=sys.stderr)
    else:
        print(json.dumps(receive_stream(sys.stdin.buffer, args.cache, args.seconds)), file=sys.stderr)


if __name__ == "__main__":
    _main()
