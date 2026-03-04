"""
ROS2 communication backend for distributed control.

Implements CommunicationBackend using ROS2 pub/sub, enabling the same
distributed algorithms to run over a real network for sim-to-real transfer.

Requires ROS2 (rclpy) to be installed and sourced. Import this module only
when ROS2 is available; backend.py remains importable without it.

Topic structure
---------------
Each agent publishes to its own outbox topic:
    /swarm/agent_{i}/outbox  (std_msgs/Float64MultiArray)

Every neighbor of agent i subscribes to /swarm/agent_i/outbox. Messages
carry an explicit to_id field so receivers can filter messages not meant
for them (multiple neighbors share one outbox topic).

Message wire format (Float64MultiArray.data)
--------------------------------------------
[ from_id, to_id, epoch, dim, eta[0], ..., eta[dim-1],
  lam[0,0], lam[0,1], ..., lam[dim-1,dim-1] ]

Total length: 4 + dim + dim² floats.

Barrier implementation
----------------------
Uses distributed epoch-counting: each agent knows exactly how many
neighbors will send to it per round. barrier() spins the SingleThreadedExecutor
until all per-agent received counts meet expectations, then increments the epoch.
No coordinator node required.

Usage
-----
    from communication.ros2_backend import ROS2Backend
    from communication.backend import create_ring_topology

    topo = create_ring_topology(4)
    backend = ROS2Backend(num_agents=4, topology=topo)
    # ... use exactly like SimulatedBackend ...
    backend.shutdown()
"""

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from std_msgs.msg import Float64MultiArray
    _ROS2_AVAILABLE = True
except ImportError:
    _ROS2_AVAILABLE = False

import numpy as np
import time
from typing import List, Tuple, Dict, Any

from .backend import CommunicationBackend, GaussianMessage


def _require_ros2() -> None:
    if not _ROS2_AVAILABLE:
        raise ImportError(
            "ROS2 (rclpy) is not installed or not sourced. "
            "Install ROS2 and source the setup script before using ROS2Backend."
        )


class ROS2Backend(CommunicationBackend):
    """
    ROS2 pub/sub message passing backend.

    Manages one rclpy.Node per agent, all within the same process. Spins a
    SingleThreadedExecutor in barrier() to deliver pending messages before
    proceeding to the next GBP round.

    Parameters
    ----------
    num_agents : int
        Number of agents in the system.
    topology : dict, optional
        Agent topology (id -> list of neighbor ids). Defaults to fully connected.
    barrier_timeout : float
        Seconds to wait in barrier() before raising TimeoutError. Default 5.0.
    namespace : str
        ROS2 topic namespace prefix. Default '/swarm'.
    """

    def __init__(
        self,
        num_agents: int,
        topology: Dict[int, List[int]] = None,
        barrier_timeout: float = 5.0,
        namespace: str = '/swarm',
    ):
        _require_ros2()
        super().__init__(num_agents, topology)

        self._barrier_timeout = barrier_timeout
        self._namespace = namespace.rstrip('/')
        self._current_epoch = 0

        # Per-agent message inbox and received-this-round counter
        self._inbox: Dict[int, List[Tuple[int, GaussianMessage]]] = {
            i: [] for i in range(num_agents)
        }
        self._received_counts: Dict[int, int] = {i: 0 for i in range(num_agents)}

        # How many messages each agent expects per round (= number of neighbors)
        # Assumes each neighbor sends exactly one message per round, matching
        # the synchronous GBP pattern where every agent broadcasts to all neighbors.
        self._expected: Dict[int, int] = {
            i: len(self.topology[i]) for i in range(num_agents)
        }

        if not rclpy.ok():
            rclpy.init()

        # Create one node per agent
        self._nodes: List[Node] = []
        self._publishers: Dict[int, Any] = {}

        for i in range(num_agents):
            node = rclpy.create_node(f'swarm_agent_{i}')
            self._nodes.append(node)
            self._publishers[i] = node.create_publisher(
                Float64MultiArray,
                f'{self._namespace}/agent_{i}/outbox',
                qos_profile=10,
            )

        # Subscriptions: agent i subscribes to each neighbor j's outbox
        for i in range(num_agents):
            for j in self.topology[i]:
                self._nodes[i].create_subscription(
                    Float64MultiArray,
                    f'{self._namespace}/agent_{j}/outbox',
                    self._make_callback(receiver_id=i),
                    qos_profile=10,
                )

        # Single executor manages all nodes; callbacks run in the calling thread
        self._executor = SingleThreadedExecutor()
        for node in self._nodes:
            self._executor.add_node(node)

    # ------------------------------------------------------------------
    # Serialization
    # ------------------------------------------------------------------

    def _serialize(
        self, from_id: int, to_id: int, message: GaussianMessage
    ) -> 'Float64MultiArray':
        """Pack a GaussianMessage into a Float64MultiArray."""
        dim = len(message.eta)
        ros_msg = Float64MultiArray()
        ros_msg.data = [
            float(from_id),
            float(to_id),
            float(message.epoch),
            float(dim),
            *message.eta.tolist(),
            *message.lam.flatten().tolist(),
        ]
        return ros_msg

    @staticmethod
    def _deserialize(data) -> Tuple[int, int, GaussianMessage]:
        """Unpack a Float64MultiArray into (from_id, to_id, GaussianMessage)."""
        from_id = int(data[0])
        to_id = int(data[1])
        epoch = int(data[2])
        dim = int(data[3])
        eta = np.array(data[4:4 + dim])
        lam = np.array(data[4 + dim:4 + dim + dim * dim]).reshape(dim, dim)
        return from_id, to_id, GaussianMessage(eta=eta, lam=lam, epoch=epoch)

    # ------------------------------------------------------------------
    # Subscription callback factory
    # ------------------------------------------------------------------

    def _make_callback(self, receiver_id: int):
        """Return a subscription callback that deposits messages into receiver_id's inbox."""
        def callback(ros_msg: 'Float64MultiArray') -> None:
            from_id, to_id, gmsg = self._deserialize(ros_msg.data)
            if to_id != receiver_id:
                return  # Message on this outbox topic is addressed to another agent
            self._inbox[receiver_id].append((from_id, gmsg))
            self._received_counts[receiver_id] += 1

        return callback

    # ------------------------------------------------------------------
    # CommunicationBackend interface
    # ------------------------------------------------------------------

    def send(self, from_id: int, to_id: int, message: Any) -> None:
        if to_id not in self.topology.get(from_id, []):
            raise ValueError(f"Agent {from_id} cannot send to non-neighbor {to_id}")

        stamped = GaussianMessage(
            eta=message.eta.copy(),
            lam=message.lam.copy(),
            epoch=self._current_epoch,
        )
        ros_msg = self._serialize(from_id, to_id, stamped)
        self._publishers[from_id].publish(ros_msg)
        self._stats['messages_sent'] += 1

    def receive(self, agent_id: int) -> List[Tuple[int, GaussianMessage]]:
        messages = self._inbox[agent_id]
        self._inbox[agent_id] = []
        self._received_counts[agent_id] = 0
        return messages

    def broadcast(self, from_id: int, message: Any) -> None:
        for neighbor in self.topology[from_id]:
            self.send(from_id, neighbor, message)

    def barrier(self) -> None:
        """
        Block until every agent has received all expected messages for this epoch.

        Spins the executor to deliver pending ROS2 callbacks. Once all
        per-agent received counts match expectations, advances the epoch.

        Raises
        ------
        TimeoutError
            If barrier_timeout seconds elapse before all messages arrive.
        """
        deadline = time.monotonic() + self._barrier_timeout

        while True:
            all_ready = all(
                self._received_counts[i] >= self._expected[i]
                for i in range(self.num_agents)
            )
            if all_ready:
                break

            if time.monotonic() > deadline:
                missing = {
                    i: self._expected[i] - self._received_counts[i]
                    for i in range(self.num_agents)
                    if self._received_counts[i] < self._expected[i]
                }
                raise TimeoutError(
                    f"ROS2Backend barrier timeout after {self._barrier_timeout}s. "
                    f"Agents still waiting (agent: messages_missing): {missing}"
                )

            self._executor.spin_once(timeout_sec=0.005)

        self._current_epoch += 1
        self._stats['barrier_calls'] += 1

    @property
    def is_synchronous(self) -> bool:
        return True

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def shutdown(self) -> None:
        """Destroy all nodes and shut down rclpy. Call when done."""
        for node in self._nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def __del__(self) -> None:
        try:
            self.shutdown()
        except Exception:
            pass


class SingleAgentROS2Backend(CommunicationBackend):
    """
    One-agent-per-process ROS2 communication backend.

    Intended for real-robot deployment where each TurtleBot3 runs its own
    ROS2 process. Creates a single rclpy Node for ``my_id``.

    Publishes outgoing GBP messages to:
        /swarm/agent_{my_id}/outbox   (Float64MultiArray)

    Subscribes to neighbors' outboxes:
        /swarm/agent_{j}/outbox   for j in neighbors

    The serialization wire format is identical to ROS2Backend, so agents
    running SingleAgentROS2Backend and ROS2Backend (multi-agent, same process)
    can interoperate on the same ROS2 network.

    Parameters
    ----------
    my_id : int
        This agent's ID.
    neighbors : list of int
        IDs of agents this agent communicates with.
    barrier_timeout : float
        Seconds to wait in barrier() before raising TimeoutError. Default 5.0.
    namespace : str
        ROS2 topic namespace prefix. Default '/swarm'.
    """

    def __init__(
        self,
        my_id: int,
        neighbors: List[int],
        barrier_timeout: float = 5.0,
        namespace: str = '/swarm',
    ):
        _require_ros2()
        # Build a minimal topology: only my_id knows its neighbors
        topology = {my_id: list(neighbors)}
        super().__init__(num_agents=1, topology=topology)

        self.my_id = my_id
        self._neighbors = list(neighbors)
        self._barrier_timeout = barrier_timeout
        self._namespace = namespace.rstrip('/')
        self._current_epoch = 0

        # Inbox and received-this-round counter
        self._inbox: List[Tuple[int, GaussianMessage]] = []
        self._received_count: int = 0
        self._expected: int = len(neighbors)  # one message per neighbor per round

        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.create_node(f'swarm_agent_{my_id}')
        self._publisher = self._node.create_publisher(
            Float64MultiArray,
            f'{self._namespace}/agent_{my_id}/outbox',
            qos_profile=10,
        )

        for j in neighbors:
            self._node.create_subscription(
                Float64MultiArray,
                f'{self._namespace}/agent_{j}/outbox',
                self._callback,
                qos_profile=10,
            )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

    # ------------------------------------------------------------------
    # Serialization (reuse ROS2Backend static methods via module-level fns)
    # ------------------------------------------------------------------

    def _serialize(self, to_id: int, message: GaussianMessage) -> 'Float64MultiArray':
        dim = len(message.eta)
        ros_msg = Float64MultiArray()
        ros_msg.data = [
            float(self.my_id),
            float(to_id),
            float(message.epoch),
            float(dim),
            *message.eta.tolist(),
            *message.lam.flatten().tolist(),
        ]
        return ros_msg

    def _callback(self, ros_msg: 'Float64MultiArray') -> None:
        from_id, to_id, gmsg = ROS2Backend._deserialize(ros_msg.data)
        if to_id != self.my_id:
            return
        self._inbox.append((from_id, gmsg))
        self._received_count += 1

    # ------------------------------------------------------------------
    # CommunicationBackend interface
    # ------------------------------------------------------------------

    def send(self, from_id: int, to_id: int, message: Any) -> None:
        if from_id != self.my_id:
            raise ValueError(f"SingleAgentROS2Backend: from_id must be {self.my_id}")
        if to_id not in self._neighbors:
            raise ValueError(f"Agent {self.my_id} cannot send to non-neighbor {to_id}")

        stamped = GaussianMessage(
            eta=message.eta.copy(),
            lam=message.lam.copy(),
            epoch=self._current_epoch,
        )
        self._publisher.publish(self._serialize(to_id, stamped))
        self._stats['messages_sent'] += 1

    def receive(self, agent_id: int) -> List[Tuple[int, GaussianMessage]]:
        messages = list(self._inbox)
        self._inbox.clear()
        self._received_count = 0
        return messages

    def broadcast(self, from_id: int, message: Any) -> None:
        for neighbor in self._neighbors:
            self.send(from_id, neighbor, message)

    def barrier(self) -> None:
        """
        Block until all expected messages for this round have arrived.

        Spins the executor to deliver pending ROS2 callbacks. Raises
        TimeoutError if barrier_timeout elapses.
        """
        deadline = time.monotonic() + self._barrier_timeout
        while self._received_count < self._expected:
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"SingleAgentROS2Backend barrier timeout after "
                    f"{self._barrier_timeout}s. "
                    f"Received {self._received_count}/{self._expected} messages."
                )
            self._executor.spin_once(timeout_sec=0.005)

        self._current_epoch += 1
        self._stats['barrier_calls'] += 1

    @property
    def is_synchronous(self) -> bool:
        return True

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def shutdown(self) -> None:
        """Destroy the node and shut down rclpy."""
        self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def __del__(self) -> None:
        try:
            self.shutdown()
        except Exception:
            pass
