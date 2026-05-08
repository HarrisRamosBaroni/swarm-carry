#!/usr/bin/env python3
"""
Pre-flight comms diagnostic for decentralised swarm deployment.

Runs four layers of checks between every robot pair:
  1. Laptop → robot ping
  2. Robot → robot ping
  3. Robot → robot TCP port reachability (nc)
  4. Robot → robot ZMQ PUB/SUB message delivery

Usage (from repo root):
    python3 real_robot/scripts/check_comms.py [--config real_robot/config/network.yaml]
"""
import argparse
import subprocess
import sys
import time
import threading
import yaml

GREEN  = "\033[32m"
RED    = "\033[31m"
YELLOW = "\033[33m"
RESET  = "\033[0m"
PASS   = f"{GREEN}PASS{RESET}"
FAIL   = f"{RED}FAIL{RESET}"
SKIP   = f"{YELLOW}SKIP{RESET}"

SSH_OPTS = "-o ConnectTimeout=5 -o StrictHostKeyChecking=no -o BatchMode=yes"
REMOTE_USER = "ubuntu"


def _run(cmd, timeout=8):
    try:
        r = subprocess.run(
            cmd, shell=True, capture_output=True, text=True, timeout=timeout,
        )
        return r.returncode == 0, r.stdout.strip(), r.stderr.strip()
    except subprocess.TimeoutExpired:
        return False, "", "timeout"


def _ssh(ip, remote_cmd, timeout=8):
    return _run(f'ssh {SSH_OPTS} {REMOTE_USER}@{ip} {remote_cmd!r}', timeout=timeout)


def _label(tag, width=40):
    return f"  {tag:<{width}}"


# ---------------------------------------------------------------------------
# Layer 1: laptop → robot ping
# ---------------------------------------------------------------------------

def check_ping_laptop_to_robot(ip):
    ok, _, _ = _run(f"ping -c 2 -W 2 {ip}", timeout=8)
    return ok


# ---------------------------------------------------------------------------
# Layer 2: robot → robot ping
# ---------------------------------------------------------------------------

def check_ping_robot_to_robot(src_ip, dst_ip):
    ok, _, _ = _ssh(src_ip, f"ping -c 2 -W 2 {dst_ip}", timeout=10)
    return ok


# ---------------------------------------------------------------------------
# Layer 3: TCP port reachability (nc)
# We bind a port on dst with socat/nc, then probe from src.
# ---------------------------------------------------------------------------

_NC_BIND = "python3.12 -c \"\
import socket, time;\
s=socket.socket();\
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1);\
s.bind(('0.0.0.0', {port}));\
s.listen(1);\
s.settimeout(5);\
s.accept();\
s.close()\""

_NC_PROBE = "nc -zv -w 3 {ip} {port}"


def check_tcp_port(src_ip, dst_ip, port, timeout=12):
    """Bind port on dst, probe from src. Returns (ok, detail)."""
    bind_cmd = _NC_BIND.format(port=port)
    probe_cmd = _NC_PROBE.format(ip=dst_ip, port=port)

    result = {"ok": False, "detail": "listener never started"}
    listener_ready = threading.Event()

    def _listen():
        # We can't signal "ready" across SSH easily, so just fire and forget.
        _ssh(dst_ip, bind_cmd, timeout=8)

    t = threading.Thread(target=_listen, daemon=True)
    t.start()
    time.sleep(1.0)  # give listener time to bind

    ok, out, err = _ssh(src_ip, probe_cmd, timeout=8)
    t.join(timeout=3)
    return ok, (out or err)


# ---------------------------------------------------------------------------
# Layer 4: ZMQ PUB/SUB end-to-end
# pub_ip binds the PUB socket; sub_ip connects and waits for a message.
# ---------------------------------------------------------------------------

_ZMQ_PUB = """python3.12 -c "
import zmq, time
ctx = zmq.Context()
s = ctx.socket(zmq.PUB)
s.bind('tcp://*:{port}')
time.sleep(0.6)
for _ in range(10):
    s.send_string('zmqtest:ping')
    time.sleep(0.1)
s.close(); ctx.term()
print('sent')
" """

_ZMQ_SUB = """python3.12 -c "
import zmq
ctx = zmq.Context()
s = ctx.socket(zmq.SUB)
s.connect('tcp://{pub_ip}:{port}')
s.setsockopt_string(zmq.SUBSCRIBE, 'zmqtest')
s.setsockopt(zmq.RCVTIMEO, 4000)
try:
    msg = s.recv_string()
    print('OK' if 'ping' in msg else 'WRONG')
except zmq.Again:
    print('TIMEOUT')
finally:
    s.close(); ctx.term()
" """


def check_zmq(pub_ip, pub_port, sub_ip, timeout=14):
    """Test ZMQ message delivery from pub_ip:pub_port to sub_ip."""
    pub_cmd = _ZMQ_PUB.format(port=pub_port)
    sub_cmd = _ZMQ_SUB.format(pub_ip=pub_ip, port=pub_port)

    sub_result = {}

    def _run_sub():
        ok, out, err = _ssh(sub_ip, sub_cmd, timeout=10)
        sub_result["ok"] = ok
        sub_result["out"] = out
        sub_result["err"] = err

    sub_thread = threading.Thread(target=_run_sub, daemon=True)
    sub_thread.start()
    time.sleep(0.4)  # let subscriber connect before publisher starts sending

    pub_ok, pub_out, pub_err = _ssh(pub_ip, pub_cmd, timeout=8)
    sub_thread.join(timeout=8)

    out = sub_result.get("out", "")
    if "OK" in out:
        return True, "message received"
    elif "TIMEOUT" in out:
        return False, "subscriber timed out (no messages arrived)"
    elif not sub_result:
        return False, "subscriber SSH failed"
    else:
        return False, f"unexpected output: {out!r} err={sub_result.get('err','')!r}"


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="real_robot/config/network.yaml")
    args = parser.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    robots = cfg["robots"]
    n = len(robots)
    all_passed = True

    print(f"\nChecking {n} robots from config: {args.config}\n")

    # Layer 1: laptop → each robot
    print("── Layer 1: Laptop → robot ping ──────────────────────────────")
    reachable = {}
    for r in robots:
        ok = check_ping_laptop_to_robot(r["ip"])
        reachable[r["id"]] = ok
        icon = PASS if ok else FAIL
        print(f"  laptop → robot {r['id']} ({r['ip']}): {icon}")
        if not ok:
            all_passed = False
    print()

    # Layers 2-4: pairwise between robots
    print("── Layer 2: Robot → robot ping ───────────────────────────────")
    inter_reachable = {}
    for i, ra in enumerate(robots):
        for rb in robots[i + 1:]:
            if not (reachable.get(ra["id"]) and reachable.get(rb["id"])):
                print(f"  {_label(f'robot {ra[\"id\"]} ↔ robot {rb[\"id\"]}')}{SKIP}  (robot unreachable)")
                continue
            ok_ab = check_ping_robot_to_robot(ra["ip"], rb["ip"])
            ok_ba = check_ping_robot_to_robot(rb["ip"], ra["ip"])
            ok = ok_ab and ok_ba
            inter_reachable[(ra["id"], rb["id"])] = ok
            icon = PASS if ok else FAIL
            details = []
            if not ok_ab: details.append(f"{ra['id']}→{rb['id']} failed")
            if not ok_ba: details.append(f"{rb['id']}→{ra['id']} failed")
            suffix = f"  ({', '.join(details)})" if details else ""
            print(f"  robot {ra['id']} ↔ robot {rb['id']}: {icon}{suffix}")
            if not ok:
                all_passed = False
    print()

    print("── Layer 3: TCP port reachability ────────────────────────────")
    for i, ra in enumerate(robots):
        for rb in robots[i + 1:]:
            pair_ok = inter_reachable.get((ra["id"], rb["id"]), False)
            for sender, receiver in [(ra, rb), (rb, ra)]:
                tag = f"robot {sender['id']} → robot {receiver['id']}:{receiver['pub_port']}"
                if not pair_ok:
                    print(f"  {_label(tag)}{SKIP}  (no inter-robot ping)")
                    continue
                ok, detail = check_tcp_port(sender["ip"], receiver["ip"], receiver["pub_port"])
                icon = PASS if ok else FAIL
                note = f"  ({detail})" if (not ok and detail) else ""
                print(f"  {_label(tag)}{icon}{note}")
                if not ok:
                    all_passed = False
    print()

    print("── Layer 4: ZMQ PUB/SUB message delivery ─────────────────────")
    for i, ra in enumerate(robots):
        for rb in robots[i + 1:]:
            pair_ok = inter_reachable.get((ra["id"], rb["id"]), False)
            for pub, sub in [(ra, rb), (rb, ra)]:
                tag = f"robot {pub['id']} PUB → robot {sub['id']} SUB (port {pub['pub_port']})"
                if not pair_ok:
                    print(f"  {_label(tag)}{SKIP}  (no inter-robot ping)")
                    continue
                ok, detail = check_zmq(pub["ip"], pub["pub_port"], sub["ip"])
                icon = PASS if ok else FAIL
                note = f"  ({detail})" if not ok else ""
                print(f"  {_label(tag)}{icon}{note}")
                if not ok:
                    all_passed = False
    print()

    if all_passed:
        print(f"{GREEN}All checks passed — comms look healthy.{RESET}\n")
    else:
        print(f"{RED}Some checks failed — see above.{RESET}\n")
        print("Common fixes:")
        print("  Ping fails:      AP client isolation — disable on router, or use wired connection")
        print("  TCP port fails:  firewall (iptables/ufw) on robot — check 'sudo iptables -L'")
        print("  ZMQ fails:       port already bound by stale process — run deploy.sh --stop first")
        sys.exit(1)


if __name__ == "__main__":
    main()
