"""Static server for the WebAssembly build, with caching disabled.

http.server's default handler lets the browser cache the 22 MB .wasm
aggressively, so a rebuild often appears to change nothing. This sends
no-store on every response, which removes that variable entirely while
developing.

Usage:  python serve.py [port]        (defaults to 8000, serves ./build)
"""

from __future__ import annotations

import http.server
import os
import sys


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self) -> None:
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()

    def log_message(self, fmt: str, *args) -> None:
        # Keep the console readable: only report failures.
        if args and str(args[1]).startswith(("4", "5")):
            super().log_message(fmt, *args)


def main() -> int:
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "build")
    if not os.path.isdir(root):
        print(f"No build directory at {root}. Run .\\build.ps1 first.", file=sys.stderr)
        return 1
    os.chdir(root)

    url = f"http://localhost:{port}/serial_plotter_wasm.html"
    print(f"Serving {root}\n  {url}\n  (caching disabled; Ctrl+C to stop)")
    http.server.HTTPServer(("127.0.0.1", port), NoCacheHandler).serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
