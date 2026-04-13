"""Round-trip tests for real_robot/transport/messages.py"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

from real_robot.transport.messages import (
    pose_msg, state_msg, force_msg, cmd_msg, peer_msg, unpack
)


def test_pose():
    d = unpack(pose_msg(2, 1.5, -0.3, 0.78))
    assert d["t"] == "pose"
    assert d["id"] == 2
    assert abs(d["x"] - 1.5) < 1e-9
    assert abs(d["y"] - -0.3) < 1e-9
    assert abs(d["theta"] - 0.78) < 1e-9
    assert "ts" in d
    print("  pose_msg      OK")


def test_state():
    d = unpack(state_msg(0, 1.0, 2.0, 0.1, -0.2, 0.5, 0.05))
    assert d["t"] == "state"
    assert d["id"] == 0
    assert abs(d["vx"] - 0.1) < 1e-9
    assert abs(d["vy"] - -0.2) < 1e-9
    assert abs(d["omega"] - 0.05) < 1e-9
    print("  state_msg     OK")


def test_force():
    readings = [{"label": "horizontal", "value": 12.3},
                {"label": "vertical",   "value": -0.4}]
    d = unpack(force_msg(1, readings))
    assert d["t"] == "force"
    assert d["id"] == 1
    assert d["readings"][0]["label"] == "horizontal"
    assert abs(d["readings"][0]["value"] - 12.3) < 1e-6
    assert abs(d["readings"][1]["value"] - -0.4) < 1e-6
    print("  force_msg     OK")


def test_cmd():
    d = unpack(cmd_msg(3, 0.5, -0.1))
    assert d["t"] == "cmd"
    assert d["id"] == 3
    assert abs(d["vx"] - 0.5) < 1e-9
    assert abs(d["vy"] - -0.1) < 1e-9
    print("  cmd_msg       OK")


def test_peer():
    payload = b"\x01\x02\x03"
    d = unpack(peer_msg(0, 1, 7, payload))
    assert d["t"] == "peer"
    assert d["from"] == 0
    assert d["to"] == 1
    assert d["epoch"] == 7
    assert d["payload"] == payload
    print("  peer_msg      OK")


if __name__ == "__main__":
    print("test_messages")
    test_pose()
    test_state()
    test_force()
    test_cmd()
    test_peer()
    print("All passed.")
