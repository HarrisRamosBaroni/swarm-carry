"""
Message schemas for real-robot ZeroMQ transport.
All messages are msgpack-serialised dicts. Import this on both laptop and robot.
"""
import time
import msgpack
import numpy as np


def pose_msg(robot_id: int, x: float, y: float, theta: float) -> bytes:
    return msgpack.packb({
        "t": "pose",
        "id": robot_id,
        "ts": time.time(),
        "x": x, "y": y, "theta": theta,
    })


def state_msg(robot_id: int, x: float, y: float,
              vx: float, vy: float, theta: float, omega: float) -> bytes:
    return msgpack.packb({
        "t": "state",
        "id": robot_id,
        "ts": time.time(),
        "x": x, "y": y, "vx": vx, "vy": vy, "theta": theta, "omega": omega,
    })


def force_msg(robot_id: int, readings: list) -> bytes:
    """
    readings: list of {"label": str, "value": float} dicts, one per load cell.
    Format TBD pending physical mounting geometry — hardware team fills this in.
    Example: [{"label": "lc_base", "value": 12.3}, {"label": "lc_wall_x", "value": -0.4}]
    """
    return msgpack.packb({
        "t": "force",
        "id": robot_id,
        "ts": time.time(),
        "readings": readings,
    })


def estop_msg() -> bytes:
    return msgpack.packb({"t": "estop", "ts": time.time()})


def ctrl_stop_msg() -> bytes:
    """Pause the active controller without killing any process.
    central_runner clears its goal and waits; agent_runners set a paused flag
    and stop executing their local controller until a new goal arrives.
    Unlike estop, nothing exits — cmd messages from the laptop still get through."""
    return msgpack.packb({"t": "ctrl_stop", "ts": time.time()})


def goal_msg(x: float, y: float, theta: float, tol: float) -> bytes:
    return msgpack.packb({
        "t": "goal",
        "x": x, "y": y, "theta": theta, "tol": tol,
    })


def cmd_msg(robot_id: int, vx: float, vy: float) -> bytes:
    return msgpack.packb({
        "t": "cmd",
        "id": robot_id,
        "vx": vx, "vy": vy,
    })


def peer_msg(from_id: int, to_id: int, epoch: int, payload: bytes) -> bytes:
    """payload is already serialised (e.g. msgpack bytes of GaussianMessage fields)."""
    return msgpack.packb({
        "t": "peer",
        "from": from_id,
        "to": to_id,
        "epoch": epoch,
        "payload": payload,
    })


def unpack(raw: bytes) -> dict:
    return msgpack.unpackb(raw, raw=False)
