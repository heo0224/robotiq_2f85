#!/usr/bin/env python3
"""Command-line control for the Robotiq 2F-85.

    ./gripper.py status
    ./gripper.py activate
    ./gripper.py open
    ./gripper.py close --force 100
    ./gripper.py move 128 --speed 100
    ./gripper.py mm 40
    ./gripper.py stop
    ./gripper.py reset
    ./gripper.py monitor
"""

import argparse
import sys
import time

from robotiq_2f85 import DEFAULT_PORT, GripperError, Robotiq2F85


def main() -> int:
    ap = argparse.ArgumentParser(description="Robotiq 2F-85 control")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--slave", type=int, default=9)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def motion_args(p):
        p.add_argument("--speed", type=int, default=255, help="0-255")
        p.add_argument("--force", type=int, default=150, help="0-255")
        p.add_argument("--no-wait", action="store_true")

    sub.add_parser("status")
    sub.add_parser("activate")
    sub.add_parser("reset")
    sub.add_parser("stop")
    motion_args(sub.add_parser("open"))
    motion_args(sub.add_parser("close"))
    p = sub.add_parser("move"); p.add_argument("position", type=int, help="0=open 255=closed"); motion_args(p)
    p = sub.add_parser("mm"); p.add_argument("opening", type=float, help="0-85 mm"); motion_args(p)
    p = sub.add_parser("monitor"); p.add_argument("--hz", type=float, default=5.0)

    args = ap.parse_args()
    kw = {}
    if hasattr(args, "speed"):
        kw = {"speed": args.speed, "force": args.force, "wait": not args.no_wait}

    try:
        with Robotiq2F85(port=args.port, slave_id=args.slave) as g:
            if args.cmd == "status":
                print(g.get_status())
            elif args.cmd == "activate":
                print(g.activate())
            elif args.cmd == "reset":
                g.reset(); print("reset (rACT cleared)")
            elif args.cmd == "stop":
                g.stop(); print(g.get_status())
            elif args.cmd == "open":
                print(g.open(**kw))
            elif args.cmd == "close":
                print(g.close(**kw))
            elif args.cmd == "move":
                print(g.move(args.position, **kw))
            elif args.cmd == "mm":
                print(g.move_mm(args.opening, **kw))
            elif args.cmd == "monitor":
                period = 1.0 / args.hz
                try:
                    while True:
                        print(f"\r{g.get_status()}   ", end="", flush=True)
                        time.sleep(period)
                except KeyboardInterrupt:
                    print()
    except GripperError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    except Exception as e:  # serial errors, bad port, ...
        print(f"error: {type(e).__name__}: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
