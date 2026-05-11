"""
Recorder — subscribes to mocap, robot, and control-panel ZMQ topics and writes
JSONL files into real_robot/recordings/. Lifecycle is driven from the control
panel via a "rec_ctrl" topic on the laptop goal port:

  start    → open a new _active_<utc>.jsonl
  stop     → close, rename to <utc>[_<name>].jsonl, write .meta.json sidecar
  discard  → close and delete

Runs on the same laptop as mocap_bridge / launch.sh so all timestamps share one
clock. Idle until "start".

Per-(topic, id) throttling keeps file size small — pose @ 30 Hz, force @ 50 Hz,
everything else unthrottled.

Usage:
  python3 -m real_robot.laptop.record --config real_robot/config/network.yaml --mode central
"""
import argparse
import json
import os
import time
from datetime import datetime, timezone
from pathlib import Path

import msgpack
import yaml
import zmq


POSE_HZ = 30.0
FORCE_HZ = 50.0
RECORDINGS_DIR = Path("real_robot/recordings")


def _utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def _safe_name(name: str) -> str:
    keep = "-_."
    return "".join(c if c.isalnum() or c in keep else "_" for c in name).strip("_")


class Recorder:
    def __init__(self, mode: str, controller: str, config_path: str, cfg: dict):
        self.mode = mode
        self.controller = controller
        self.config_path = config_path
        self.cfg = cfg
        self.active_path: Path | None = None
        self.active_fh = None
        self.started_at: float | None = None
        self.row_count = 0
        self.last_logged: dict[tuple, float] = {}
        RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)

    @property
    def recording(self) -> bool:
        return self.active_fh is not None

    def start(self) -> None:
        if self.recording:
            print("[record] start ignored — already recording")
            return
        stamp = _utc_stamp()
        self.active_path = RECORDINGS_DIR / f"_active_{stamp}.jsonl"
        self.active_fh = open(self.active_path, "w", buffering=1)  # line-buffered
        self.started_at = time.time()
        self.row_count = 0
        self.last_logged.clear()
        print(f"[record] START → {self.active_path}")

    def stop(self, name: str, notes: str) -> None:
        if not self.recording:
            print("[record] stop ignored — not recording")
            return
        self.active_fh.close()
        active = self.active_path
        stamp = active.stem.replace("_active_", "", 1)
        safe = _safe_name(name)
        final_name = f"{stamp}_{safe}.jsonl" if safe else f"{stamp}.jsonl"
        final_path = RECORDINGS_DIR / final_name
        active.rename(final_path)
        meta = {
            "stamp_utc": stamp,
            "name": name,
            "notes": notes,
            "mode": self.mode,
            "controller": self.controller,
            "config_path": self.config_path,
            "started_at": self.started_at,
            "stopped_at": time.time(),
            "duration_s": time.time() - (self.started_at or time.time()),
            "row_count": self.row_count,
        }
        meta_path = final_path.with_suffix(".meta.json")
        with open(meta_path, "w") as f:
            json.dump(meta, f, indent=2)
        print(f"[record] STOP  → {final_path} ({self.row_count} rows, "
              f"{meta['duration_s']:.1f}s)")
        self.active_fh = None
        self.active_path = None
        self.started_at = None

    def discard(self) -> None:
        if not self.recording:
            print("[record] discard ignored — not recording")
            return
        self.active_fh.close()
        try:
            self.active_path.unlink()
        except FileNotFoundError:
            pass
        print(f"[record] DISCARDED ({self.row_count} rows dropped)")
        self.active_fh = None
        self.active_path = None
        self.started_at = None

    def write(self, topic: str, data: dict) -> None:
        if not self.recording:
            return
        # Per-(topic, id) throttle.
        rid = data.get("id") if isinstance(data, dict) else None
        key = (topic, rid)
        interval = None
        if topic == "pose":
            interval = 1.0 / POSE_HZ
        elif topic == "force":
            interval = 1.0 / FORCE_HZ
        if interval is not None:
            now = time.monotonic()
            last = self.last_logged.get(key, 0.0)
            if now - last < interval:
                return
            self.last_logged[key] = now
        row = {"t": time.time(), "topic": topic, "data": data}
        self.active_fh.write(json.dumps(row, default=str) + "\n")
        self.row_count += 1


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", default="real_robot/config/network.yaml")
    p.add_argument("--mode", default="unknown",
                   help="central|decentralised — recorded into meta only")
    p.add_argument("--controller", default="unknown",
                   help="controller name — recorded into meta only")
    args = p.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    # Robot pub_ports carry "force".
    for r in cfg.get("robots", []):
        sub.connect(f"tcp://{r['ip']}:{r['pub_port']}")
    laptop = cfg["laptop"]
    # mocap → "pose".
    sub.connect(f"tcp://{laptop['ip']}:{laptop['mocap_pub_port']}")
    # central_runner → "cmd".
    if "central_pub_port" in laptop:
        sub.connect(f"tcp://{laptop['ip']}:{laptop['central_pub_port']}")
    # control_panel → "goal", "estop", "ctrl_stop", "rec_ctrl".
    if "goal_pub_port" in laptop:
        sub.connect(f"tcp://{laptop['ip']}:{laptop['goal_pub_port']}")

    for topic in (b"pose", b"force", b"cmd", b"goal", b"estop",
                  b"ctrl_stop", b"rec_ctrl"):
        sub.setsockopt(zmq.SUBSCRIBE, topic)

    rec = Recorder(args.mode, args.controller, args.config, cfg)
    print(f"[record] ready — mode={args.mode}, controller={args.controller}")
    print(f"[record] output dir: {RECORDINGS_DIR.resolve()}")
    print("[record] idle — waiting for 'start' from control panel")

    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)

    try:
        while True:
            events = dict(poller.poll(200))
            if sub not in events:
                continue
            try:
                topic_b, raw = sub.recv_multipart()
            except ValueError:
                continue
            topic = topic_b.decode("utf-8", errors="replace")
            try:
                data = msgpack.unpackb(raw, raw=False)
            except Exception as e:
                data = {"_unpack_error": str(e)}

            if topic == "rec_ctrl":
                action = data.get("action") if isinstance(data, dict) else None
                if action == "start":
                    rec.start()
                elif action == "stop":
                    rec.stop(data.get("name", ""), data.get("notes", ""))
                elif action == "discard":
                    rec.discard()
                else:
                    print(f"[record] unknown rec_ctrl action: {action}")
                continue

            rec.write(topic, data)
    except KeyboardInterrupt:
        print("\n[record] interrupted")
        if rec.recording:
            rec.stop(name="interrupted", notes="recorder killed via Ctrl+C")


if __name__ == "__main__":
    main()
