"""Bounded file responses shared by result replay endpoints."""
from http import HTTPStatus


def serve_file(self, path, *, head=False, content_type="video/mp4"):
    with path.open("rb") as stream:
        size = path.stat().st_size
        start, end = 0, size - 1
        requested = self.headers.get("Range")
        if requested:
            try:
                unit, value = requested.split("=", 1)
                first, last = value.split("-", 1)
                if unit != "bytes" or "," in value or not (first or last):
                    raise ValueError()
                if first:
                    start = int(first)
                    end = min(int(last), size - 1) if last else size - 1
                else:
                    suffix = int(last)
                    if suffix <= 0:
                        raise ValueError()
                    start = max(0, size - suffix)
                if start < 0 or start >= size or end < start:
                    raise ValueError()
            except ValueError:
                self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                self.send_header("Content-Range", f"bytes */{size}")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
        self.send_response(HTTPStatus.PARTIAL_CONTENT if requested else HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(end - start + 1))
        if requested:
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.end_headers()
        if not head:
            stream.seek(start)
            remaining = end - start + 1
            try:
                while remaining:
                    block = stream.read(min(65536, remaining))
                    if not block:
                        break
                    self.wfile.write(block)
                    remaining -= len(block)
            except (BrokenPipeError, ConnectionResetError):
                pass
