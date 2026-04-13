"""
Integration test for ZeroMQSingleAgentBackend.

Runs two backends (robot 0 and robot 1) in threads on localhost,
sends a GaussianMessage from each to the other, and checks receipt.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

import threading
import numpy as np
from swarmlib.communication.zmq_backend import ZeroMQSingleAgentBackend
from swarmlib.communication.backend import GaussianMessage

# Sync point: both backends must be constructed before either sends,
# otherwise ZeroMQ PUB drops messages before the peer SUB has connected.
_ready = threading.Barrier(2)

# Minimal network config — both robots on localhost with non-clashing ports
LOOPBACK_CONFIG = {
    "robots": [
        {"id": 0, "ip": "127.0.0.1", "pub_port": 15550},
        {"id": 1, "ip": "127.0.0.1", "pub_port": 15551},
    ]
}

results = {}


def run_robot(my_id, neighbor_id, send_eta):
    b = ZeroMQSingleAgentBackend(
        my_id=my_id,
        neighbors=[neighbor_id],
        network_config=LOOPBACK_CONFIG,
        barrier_timeout=5.0,
    )
    _ready.wait()  # both PUB sockets bound before either sends
    import time; time.sleep(0.3)  # let ZMQ TCP connections finish on loopback
    msg = GaussianMessage(eta=np.array(send_eta), lam=np.eye(2))
    b.broadcast(my_id, msg)
    b.barrier()
    received = b.receive(my_id)
    results[my_id] = received
    b.shutdown()


def test_send_receive():
    t0 = threading.Thread(target=run_robot, args=(0, 1, [1.0, 2.0]))
    t1 = threading.Thread(target=run_robot, args=(1, 0, [3.0, 4.0]))
    t0.start(); t1.start()
    t0.join(timeout=8); t1.join(timeout=8)

    assert 0 in results and 1 in results, "One or both backends timed out"

    msgs0 = results[0]
    msgs1 = results[1]

    assert len(msgs0) == 1, f"Robot 0 expected 1 message, got {len(msgs0)}"
    assert len(msgs1) == 1, f"Robot 1 expected 1 message, got {len(msgs1)}"

    from_id_0, gmsg_0 = msgs0[0]
    from_id_1, gmsg_1 = msgs1[0]

    assert from_id_0 == 1, f"Robot 0 expected msg from 1, got from {from_id_0}"
    assert from_id_1 == 0, f"Robot 1 expected msg from 0, got from {from_id_1}"

    assert np.allclose(gmsg_0.eta, [3.0, 4.0]), f"Wrong eta at robot 0: {gmsg_0.eta}"
    assert np.allclose(gmsg_1.eta, [1.0, 2.0]), f"Wrong eta at robot 1: {gmsg_1.eta}"

    print("  send/receive  OK")
    print(f"    robot 0 got eta={gmsg_0.eta} from robot {from_id_0}")
    print(f"    robot 1 got eta={gmsg_1.eta} from robot {from_id_1}")


if __name__ == "__main__":
    print("test_zmq_backend")
    test_send_receive()
    print("All passed.")
