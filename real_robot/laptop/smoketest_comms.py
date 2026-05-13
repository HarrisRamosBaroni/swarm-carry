"""
Wire-level comms smoketest — run from the laptop. Tells you which layer of the
ZMQ stack is broken without launching the full pipeline.

Probes every endpoint in network.yaml:
  - laptop:mocap_pub_port   (mocap_bridge "pose")
  - laptop:central_pub_port ("cmd" from central_runner)
  - laptop:goal_pub_port    ("goal"/"estop"/"ctrl_stop" from control_panel)
  - <robot ip>:<robot pub_port>  ("force", "cmd" (decentralised), "peer:*:*")

For each endpoint it does two things:
  1. TCP-connect probe — is the port reachable at all?
  2. ZMQ SUB sniff for `--duration` seconds — what topics arrive, at what rate?

Then it interprets each row and prints a verdict.

Usage:
  python3 -m real_robot.laptop.smoketest_comms
  python3 -m real_robot.laptop.smoketest_comms --duration 8 --mode decentralised
"""
import argparse
import socket
import time
from collections import defaultdict

import yaml
import zmq


def _tcp_reachable(ip: str, port: int, timeout: float = 1.0) -> bool:
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except OSError:
        return False


def _classify_robot(endpoint: str, topics: dict, mode: str, expected_peers: int) -> str:
    if not topics:
        return "✗ silent — agent_runner not running (or wrong port / firewall)"
    force = topics.get("force", 0)
    peer_count = sum(c for t, c in topics.items() if t.startswith("peer:"))
    cmd = topics.get("cmd", 0)
    parts = []
    if force == 0:
        parts.append("no force — load cell loop not ticking?")
    else:
        parts.append(f"force ok ({force})")
    if mode == "decentralised":
        if peer_count == 0:
            parts.append("✗ NO peer:*:* — controller compute loop never reached "
                        "backend.broadcast (no goal? paused? peer-pose missing?)")
        else:
            parts.append(f"peer ok ({peer_count})")
        if cmd == 0:
            parts.append("no cmd published — agent never issued a non-stop command")
        else:
            parts.append(f"cmd ok ({cmd})")
    else:  # central
        # In central mode, robots only forward; they should not be running GBP.
        if peer_count > 0:
            parts.append(f"⚠ unexpected peer:*:* ({peer_count}) — robot is running "
                         "GBP under central mode?")
    return " · ".join(parts)


def _classify_laptop(label: str, topics: dict, mode: str) -> str:
    if label == "mocap_pub_port":
        if topics.get("pose", 0) == 0:
            return "✗ no pose — mocap_pub / mocap_bridge not running on laptop"
        return f"pose ok ({topics.get('pose', 0)})"
    if label == "central_pub_port":
        if mode == "central":
            if topics.get("cmd", 0) == 0:
                return "✗ no cmd — central_runner not commanding (no goal? paused?)"
            return f"cmd ok ({topics.get('cmd', 0)})"
        else:
            return "— (expected silent in decentralised mode)"
    if label == "goal_pub_port":
        if not topics:
            return "— (silent until control_panel publishes a goal/estop)"
        return " · ".join(f"{t}({c})" for t, c in topics.items())
    return ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="real_robot/config/network.yaml")
    ap.add_argument("--mode", choices=["central", "decentralised"],
                    default="decentralised",
                    help="affects verdict interpretation only — doesn't change probing")
    ap.add_argument("--duration", type=float, default=5.0,
                    help="seconds to sniff each endpoint (default 5)")
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    laptop = cfg["laptop"]
    robots = cfg.get("robots", [])
    expected_peers = max(0, len(robots) - 1)

    endpoints = []  # list of (label, ip, port, kind)
    for key in ("mocap_pub_port", "central_pub_port", "goal_pub_port"):
        if key in laptop:
            endpoints.append((f"laptop.{key}", laptop["ip"], laptop[key], "laptop"))
    for r in robots:
        endpoints.append((f"robot{r['id']}.pub_port", r["ip"], r["pub_port"], "robot"))

    print(f"\n[smoketest] config: {args.config}  mode: {args.mode}  "
          f"duration: {args.duration}s  endpoints: {len(endpoints)}\n")

    # ---- Step 1: TCP reachability ------------------------------------------
    print("--- TCP reachability ---")
    reach = {}
    for label, ip, port, _ in endpoints:
        ok = _tcp_reachable(ip, port)
        reach[label] = ok
        print(f"  {label:30s} {ip}:{port:<6d} {'✓' if ok else '✗ unreachable'}")
    print()

    # ---- Step 2: ZMQ sniff -------------------------------------------------
    ctx = zmq.Context()
    sockets = {}
    for label, ip, port, _ in endpoints:
        if not reach[label]:
            continue
        s = ctx.socket(zmq.SUB)
        s.connect(f"tcp://{ip}:{port}")
        s.setsockopt_string(zmq.SUBSCRIBE, "")
        sockets[label] = s

    if not sockets:
        print("[smoketest] no reachable endpoints — nothing to sniff. Fix IPs/ports first.")
        return

    poller = zmq.Poller()
    for s in sockets.values():
        poller.register(s, zmq.POLLIN)

    # collapse peer subtopics into "peer:*" for the summary
    counts = defaultdict(lambda: defaultdict(int))   # counts[label][topic_key]
    print(f"--- sniffing for {args.duration:.1f}s ---")
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        events = dict(poller.poll(int((deadline - time.monotonic()) * 1000) or 1))
        for s, _ in events.items():
            label = next(L for L, sk in sockets.items() if sk is s)
            try:
                parts = s.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                continue
            topic = parts[0].decode("utf-8", errors="replace") if parts else "?"
            # Bucket peer:to:from → keep full key so we can count distinct edges
            counts[label][topic] += 1

    for s in sockets.values():
        s.close(linger=0)
    ctx.term()

    # ---- Step 3: report ----------------------------------------------------
    print("\n--- per-endpoint summary ---")
    for label, ip, port, kind in endpoints:
        if not reach[label]:
            print(f"\n{label}  ({ip}:{port})\n  ✗ unreachable — skipped")
            continue
        topics = dict(counts[label])
        print(f"\n{label}  ({ip}:{port})")
        if not topics:
            print("  (silent)")
        else:
            for t in sorted(topics):
                rate = topics[t] / args.duration
                print(f"  {t:30s} {topics[t]:5d} msgs  ({rate:.1f} Hz)")
        if kind == "robot":
            verdict = _classify_robot(label, topics, args.mode, expected_peers)
        else:
            key = label.split(".", 1)[1]
            verdict = _classify_laptop(key, topics, args.mode)
        print(f"  verdict: {verdict}")

    # ---- Step 4: cross-cutting checks --------------------------------------
    print("\n--- cross checks ---")
    # Peer mesh closure: in decentralised mode, for every (sender, receiver)
    # pair, sender's pub_port should emit "peer:<receiver>:<sender>:..." messages.
    if args.mode == "decentralised":
        edges_seen = set()
        for label, topics in counts.items():
            if not label.startswith("robot"):
                continue
            sender_id = int(label.split(".")[0].replace("robot", ""))
            for t in topics:
                if t.startswith("peer:"):
                    try:
                        _, to_id, from_id = t.split(":", 2)
                        edges_seen.add((int(from_id), int(to_id)))
                    except ValueError:
                        pass
        expected_edges = {(r["id"], q["id"]) for r in robots for q in robots
                          if r["id"] != q["id"]}
        missing = expected_edges - edges_seen
        if not expected_edges:
            print("  peer mesh: only one robot configured — nothing to check")
        elif not missing:
            print(f"  peer mesh: all {len(expected_edges)} directed edges observed ✓")
        else:
            print(f"  peer mesh: missing {len(missing)} / {len(expected_edges)} edges:")
            for (frm, to) in sorted(missing):
                print(f"    {frm} → {to}  (robot {frm} never published peer:{to}:{frm}:*)")
            print("  → fix: make sure each robot's agent_runner is running, has a goal, "
                  "and has --neighbors including the missing target.")


if __name__ == "__main__":
    main()
