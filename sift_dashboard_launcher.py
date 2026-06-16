#!/usr/bin/env python3
from __future__ import annotations

import argparse

from sau_sift_nav.launcher_api import run_launcher


def main() -> int:
    parser = argparse.ArgumentParser(description="SAU SIFT configurable dashboard launcher")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8099)
    parser.add_argument("--static-dir", default="dashboard/dist")
    args = parser.parse_args()
    run_launcher(host=args.host, port=args.port, static_dir=args.static_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
