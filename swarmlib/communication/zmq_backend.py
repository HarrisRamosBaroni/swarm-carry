"""
ZeroMQ single-agent communication backend.

Drop-in replacement for SingleAgentROS2Backend for real-robot deployment.
Implements CommunicationBackend using ZeroMQ PUB/SUB. No ROS dependency.

Usage (on each robot):
    from swarmlib.communication.zmq_backend import ZeroMQSingleAgentBackend
    backend = ZeroMQSingleAgentBackend(
        my_id=0,
        neighbors=[1, 2],
        network_config=config,   # loaded from real_robot/config/network.yaml
    )
    # API is identical to SingleAgentROS2Backend / SimulatedBackend
"""
import threading
import time
import msgpack
import numpy as np
import zmq
from typing import List, Tuple, Dict, Any

from swarmlib.communication.backend import CommunicationBackend, GaussianMessage


class ZeroMQSingleAgentBackend(CommunicationBackend):
    """
    One-agent-per-process ZeroMQ communication backend.

    Each robot process binds a PUB socket on its own port and connects
    SUB sockets to each neighbor's PUB port. GaussianMessage peer messages
    are serialised with msgpack.

    Parameters
    ----------
    my_id : int
    neighbors : list of int
    network_config : dict
        Parsed real_robot/config/network.yaml. Used to look up IPs and ports.
    barrier_timeout : float
        Seconds to wait in barrier() before raising TimeoutError.
    synchronous : bool
        If True (default), barrier() blocks until one message per neighbor has
        arrived for the current epoch (synchronous GBP). If False, barrier()
        does a non-blocking poll-and-drain and advances the epoch immediately —
        the caller uses whichever neighbor beliefs happen to have arrived,
        matching the asynchronous GBP scheme in the DR.CAP paper.
    """

    def __init__(
        self,
        my_id: int,
        neighbors: List[int],
        network_config: dict,
        barrier_timeout: float = 5.0,
        synchronous: bool = True,
    ):
        topology = {my_id: list(neighbors)}
        super().__init__(num_agents=1, topology=topology)

        self.my_id = my_id
        self._neighbors = list(neighbors)
        self._barrier_timeout = barrier_timeout
        self._synchronous = synchronous
        self._current_epoch = 0
        self._inbox: List[Tuple[int, GaussianMessage]] = []
        self._received_count = 0
        self._expected = len(neighbors)

        ctx = zmq.Context.instance()

        # Bind PUB socket on our own port
        my_port = _robot_port(network_config, my_id)
        self._pub = ctx.socket(zmq.PUB)
        self._pub.bind(f"tcp://*:{my_port}")

        # Connect SUB sockets to each neighbor
        self._subs: List[zmq.Socket] = []
        for nid in neighbors:
            sub = ctx.socket(zmq.SUB)
            nip = _robot_ip(network_config, nid)
            nport = _robot_port(network_config, nid)
            sub.connect(f"tcp://{nip}:{nport}")
            sub.setsockopt_string(zmq.SUBSCRIBE, f"peer:{my_id}:")
            self._subs.append(sub)

        # Poller for non-blocking receive
        self._poller = zmq.Poller()
        for sub in self._subs:
            self._poller.register(sub, zmq.POLLIN)

        self._interrupt_event = threading.Event()

        time.sleep(0.1)  # allow ZeroMQ connections to establish

    # --- Serialisation -------------------------------------------------------

    def _pack(self, to_id: int, message: GaussianMessage) -> bytes:
        return msgpack.packb({
            "from": self.my_id,
            "to": to_id,
            "epoch": message.epoch,
            "eta": message.eta.tolist(),
            "lam": message.lam.tolist(),
        })

    @staticmethod
    def _unpack(raw: bytes) -> Tuple[int, int, GaussianMessage]:
        d = msgpack.unpackb(raw, raw=False)
        return (
            d["from"],
            d["to"],
            GaussianMessage(
                eta=np.array(d["eta"]),
                lam=np.array(d["lam"]),
                epoch=d["epoch"],
            ),
        )

    # --- CommunicationBackend interface --------------------------------------

    def send(self, from_id: int, to_id: int, message: Any) -> None:
        if from_id != self.my_id:
            raise ValueError(f"from_id must be {self.my_id}")
        stamped = GaussianMessage(
            eta=message.eta.copy(), lam=message.lam.copy(),
            epoch=self._current_epoch,
        )
        topic = f"peer:{to_id}:{self.my_id}".encode()
        self._pub.send_multipart([topic, self._pack(to_id, stamped)])
        self._stats["messages_sent"] += 1

    def receive(self, agent_id: int) -> List[Tuple[int, GaussianMessage]]:
        # Drain any waiting messages before returning
        while True:
            ready = dict(self._poller.poll(timeout=0))
            if not ready:
                break
            for sub in self._subs:
                if sub in ready:
                    _, raw = sub.recv_multipart()
                    from_id, to_id, gmsg = self._unpack(raw)
                    if to_id == self.my_id:
                        self._inbox.append((from_id, gmsg))
                        self._received_count += 1

        messages = list(self._inbox)
        self._inbox.clear()
        self._received_count = 0
        return messages

    def broadcast(self, from_id: int, message: Any) -> None:
        for nid in self._neighbors:
            self.send(from_id, nid, message)

    def barrier(self) -> None:
        if not self._synchronous:
            # Async mode: drain any available messages and advance epoch.
            # The controller uses whatever arrived; missing neighbors are
            # handled by the GBP step skipping absent beliefs.
            ready = dict(self._poller.poll(timeout=0))
            for sub in self._subs:
                if sub in ready:
                    _, raw = sub.recv_multipart()
                    from_id, to_id, gmsg = self._unpack(raw)
                    if to_id == self.my_id:
                        self._inbox.append((from_id, gmsg))
                        self._received_count += 1
            self._current_epoch += 1
            self._stats["barrier_calls"] += 1
            return

        deadline = time.monotonic() + self._barrier_timeout
        while self._received_count < self._expected:
            if self._interrupt_event.is_set():
                raise InterruptedError("barrier interrupted by stop signal")
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"ZeroMQSingleAgentBackend barrier timeout. "
                    f"Received {self._received_count}/{self._expected}."
                )
            ready = dict(self._poller.poll(timeout=5))
            for sub in self._subs:
                if sub in ready:
                    _, raw = sub.recv_multipart()
                    from_id, to_id, gmsg = self._unpack(raw)
                    if to_id == self.my_id:
                        self._inbox.append((from_id, gmsg))
                        self._received_count += 1
        self._current_epoch += 1
        self._stats["barrier_calls"] += 1

    def interrupt(self) -> None:
        """Unblock a synchronous barrier() immediately."""
        self._interrupt_event.set()

    def clear_interrupt(self) -> None:
        self._interrupt_event.clear()

    @property
    def is_synchronous(self) -> bool:
        return self._synchronous

    def shutdown(self) -> None:
        self._pub.close()
        for sub in self._subs:
            sub.close()


# --- Helpers -----------------------------------------------------------------

def _robot_ip(cfg: dict, robot_id: int) -> str:
    for r in cfg["robots"]:
        if r["id"] == robot_id:
            return r["ip"]
    raise KeyError(f"Robot {robot_id} not in network config")


def _robot_port(cfg: dict, robot_id: int) -> int:
    for r in cfg["robots"]:
        if r["id"] == robot_id:
            return r["pub_port"]
    raise KeyError(f"Robot {robot_id} not in network config")
