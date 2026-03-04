"""
Communication backend interface for distributed control.

Provides swappable backends for agent-to-agent message passing:
- SimulatedBackend: Zero-overhead, for benchmarking
- (Future) ROS2Backend: Real pub/sub, for sim-to-real
"""

from abc import ABC, abstractmethod
from typing import List, Tuple, Dict, Any
from dataclasses import dataclass
import numpy as np
import time


@dataclass
class GaussianMessage:
    """
    Gaussian belief message for GBP.

    Represents N(mean, cov) or equivalently N^{-1}(eta, lam) in information form.
    Information form is more convenient for message passing:
        eta = Λ @ μ  (information vector)
        lam = Λ      (precision matrix, i.e., inverse covariance)
    """
    eta: np.ndarray   # Information vector (precision @ mean)
    lam: np.ndarray   # Precision matrix (inverse covariance)
    epoch: int = 0    # GBP round this message was sent in (for async backends)

    @classmethod
    def from_moments(cls, mean: np.ndarray, cov: np.ndarray) -> 'GaussianMessage':
        """Create from moment form (mean, covariance)."""
        lam = np.linalg.inv(cov)
        eta = lam @ mean
        return cls(eta=eta, lam=lam)

    def to_moments(self) -> Tuple[np.ndarray, np.ndarray]:
        """Convert to moment form (mean, covariance)."""
        cov = np.linalg.inv(self.lam)
        mean = cov @ self.eta
        return mean, cov

    @property
    def mean(self) -> np.ndarray:
        """Recover mean from information form."""
        return np.linalg.solve(self.lam, self.eta)

    @property
    def precision(self) -> np.ndarray:
        """Precision matrix (same as lam)."""
        return self.lam

    def __add__(self, other: 'GaussianMessage') -> 'GaussianMessage':
        """Fuse two Gaussian messages (product in distribution space)."""
        return GaussianMessage(
            eta=self.eta + other.eta,
            lam=self.lam + other.lam
        )

    def copy(self) -> 'GaussianMessage':
        return GaussianMessage(eta=self.eta.copy(), lam=self.lam.copy(), epoch=self.epoch)


class CommunicationBackend(ABC):
    """Abstract interface for agent-to-agent communication."""

    def __init__(self, num_agents: int, topology: Dict[int, List[int]] = None):
        """
        Initialize backend.

        Args:
            num_agents: Number of agents in the system
            topology: Dict mapping agent_id -> list of neighbor ids.
                      If None, defaults to fully connected.
        """
        self.num_agents = num_agents
        if topology is None:
            # Fully connected by default
            self.topology = {
                i: [j for j in range(num_agents) if j != i]
                for i in range(num_agents)
            }
        else:
            self.topology = topology

        self._stats = {
            'messages_sent': 0,
            'total_bytes': 0,
            'barrier_calls': 0,
        }

    def get_neighbors(self, agent_id: int) -> List[int]:
        """Get list of neighbors for an agent."""
        return self.topology.get(agent_id, [])

    @abstractmethod
    def send(self, from_id: int, to_id: int, message: Any) -> None:
        """
        Send a message from one agent to another.

        Args:
            from_id: Sender agent ID
            to_id: Receiver agent ID
            message: Message payload (GaussianMessage, np.ndarray, etc.)
        """
        pass

    @abstractmethod
    def receive(self, agent_id: int) -> List[Tuple[int, Any]]:
        """
        Receive all pending messages for an agent.

        Args:
            agent_id: Receiving agent ID

        Returns:
            List of (sender_id, message) tuples
        """
        pass

    @abstractmethod
    def broadcast(self, from_id: int, message: Any) -> None:
        """Send message to all neighbors."""
        pass

    @abstractmethod
    def barrier(self) -> None:
        """
        Synchronization barrier.

        Blocks until all agents have reached this point.
        For simulated backend, this is a no-op.
        For real backends, ensures message delivery before proceeding.
        """
        pass

    @property
    def is_synchronous(self) -> bool:
        """Whether barrier() provides true round synchronization.

        Sync backends: barrier() ensures all round-k messages are delivered
        before any agent proceeds to round k+1.

        Async backends: barrier() is a no-op; algorithms must tolerate
        stale or missing messages.
        """
        return True

    def get_stats(self) -> Dict[str, Any]:
        """Get communication statistics."""
        return self._stats.copy()

    def reset_stats(self) -> None:
        """Reset statistics counters."""
        self._stats = {
            'messages_sent': 0,
            'total_bytes': 0,
            'barrier_calls': 0,
        }


class SimulatedBackend(CommunicationBackend):
    """
    Zero-overhead simulated message passing.

    Messages are stored in Python dicts with no serialization or network delay.
    Use this for benchmarking to isolate algorithm performance from communication.
    """

    def __init__(self, num_agents: int, topology: Dict[int, List[int]] = None):
        super().__init__(num_agents, topology)
        self._mailboxes: Dict[int, List[Tuple[int, Any]]] = {
            i: [] for i in range(num_agents)
        }
        self._pending: Dict[int, List[Tuple[int, Any]]] = {
            i: [] for i in range(num_agents)
        }

    def send(self, from_id: int, to_id: int, message: Any) -> None:
        """Send message (stored in pending buffer until barrier)."""
        if to_id not in self.topology.get(from_id, []):
            raise ValueError(f"Agent {from_id} cannot send to non-neighbor {to_id}")

        # Deep copy to simulate actual message passing
        if isinstance(message, GaussianMessage):
            msg_copy = message.copy()
        elif isinstance(message, np.ndarray):
            msg_copy = message.copy()
        else:
            msg_copy = message  # Assume immutable or handle elsewhere

        self._pending[to_id].append((from_id, msg_copy))
        self._stats['messages_sent'] += 1

    def receive(self, agent_id: int) -> List[Tuple[int, Any]]:
        """Receive all messages from mailbox (populated by barrier)."""
        messages = self._mailboxes[agent_id]
        self._mailboxes[agent_id] = []
        return messages

    def broadcast(self, from_id: int, message: Any) -> None:
        """Send to all neighbors."""
        for neighbor in self.topology[from_id]:
            self.send(from_id, neighbor, message)

    def barrier(self) -> None:
        """
        Synchronization point: move pending messages to mailboxes.

        This simulates the "all messages sent in round k are received in round k+1"
        semantics of synchronous distributed algorithms.
        """
        # Move all pending to mailboxes
        for agent_id in range(self.num_agents):
            self._mailboxes[agent_id].extend(self._pending[agent_id])
            self._pending[agent_id] = []

        self._stats['barrier_calls'] += 1


class AsyncSimulatedBackend(CommunicationBackend):
    """
    Async simulated backend for testing robustness to message dropout and delay.

    Unlike SimulatedBackend, barrier() is a no-op — it does not synchronize
    message delivery. Instead, each message is independently scheduled for
    delivery at a future step, or dropped entirely.

    Parameters
    ----------
    num_agents : int
        Number of agents.
    topology : dict, optional
        Agent topology. Defaults to fully connected.
    dropout_rate : float
        Probability [0, 1) that any given message is silently dropped.
        dropout_rate=0.9 matches the DR.CAP worst-case experiment.
    mean_delay_steps : int
        Mean number of extra rounds before a message is delivered,
        sampled from Poisson(mean_delay_steps). 0 = no extra delay
        (message available from the very next receive() call).
    seed : int, optional
        Random seed for reproducibility.

    Notes
    -----
    Messages carry a GaussianMessage.epoch tag stamped at send time.
    The algorithm layer can use this to identify and handle stale messages
    (e.g. discard messages from epoch k-2 when processing epoch k).

    Relationship to dropout:
    - With mean_delay_steps=0 and a "discard stale" policy, async is
      equivalent to dropout (late messages that arrive out-of-epoch are
      treated as lost).
    - With a "use stale" policy, async is strictly better than equivalent
      dropout, since stale messages still carry valid (if old) information.
    """

    def __init__(
        self,
        num_agents: int,
        topology: Dict[int, List[int]] = None,
        dropout_rate: float = 0.0,
        mean_delay_steps: int = 0,
        seed: int = None,
    ):
        super().__init__(num_agents, topology)
        self._dropout_rate = dropout_rate
        self._mean_delay_steps = mean_delay_steps
        self._rng = np.random.default_rng(seed)
        self._step = 0          # Advances on each barrier() call
        self._current_epoch = 0  # Mirrors _step; stamped onto outgoing messages

        # Delivery queue: list of (delivery_step, to_id, from_id, message)
        self._queue: List[Tuple[int, int, int, Any]] = []

        # Extra stats beyond the base class
        self._stats['messages_dropped'] = 0

    @property
    def is_synchronous(self) -> bool:
        return False

    def send(self, from_id: int, to_id: int, message: Any) -> None:
        if to_id not in self.topology.get(from_id, []):
            raise ValueError(f"Agent {from_id} cannot send to non-neighbor {to_id}")

        self._stats['messages_sent'] += 1

        # Dropout: message lost in transit
        if self._rng.random() < self._dropout_rate:
            self._stats['messages_dropped'] += 1
            return

        # Sample delivery delay (Poisson; 0 = arrives this step)
        delay = int(self._rng.poisson(self._mean_delay_steps)) if self._mean_delay_steps > 0 else 0

        stamped = GaussianMessage(
            eta=message.eta.copy(),
            lam=message.lam.copy(),
            epoch=self._current_epoch,
        )
        self._queue.append((self._step + delay, to_id, from_id, stamped))

    def receive(self, agent_id: int) -> List[Tuple[int, Any]]:
        """Return all messages addressed to agent_id whose delivery step has passed."""
        ready = [
            (from_id, msg)
            for (delivery_step, to_id, from_id, msg) in self._queue
            if to_id == agent_id and delivery_step <= self._step
        ]
        self._queue = [
            entry for entry in self._queue
            if not (entry[1] == agent_id and entry[0] <= self._step)
        ]
        return ready

    def broadcast(self, from_id: int, message: Any) -> None:
        for neighbor in self.topology[from_id]:
            self.send(from_id, neighbor, message)

    def barrier(self) -> None:
        """
        No-op synchronization point.

        Advances the internal step counter so delayed messages become
        eligible for delivery in future receive() calls. Does NOT wait
        for any messages to arrive.
        """
        self._step += 1
        self._current_epoch += 1
        self._stats['barrier_calls'] += 1


def create_ring_topology(n: int) -> Dict[int, List[int]]:
    """Create ring topology: each agent connected to left and right neighbors."""
    return {i: [(i - 1) % n, (i + 1) % n] for i in range(n)}


def create_line_topology(n: int) -> Dict[int, List[int]]:
    """Create line topology: agents in a line, endpoints have one neighbor."""
    topology = {}
    for i in range(n):
        neighbors = []
        if i > 0:
            neighbors.append(i - 1)
        if i < n - 1:
            neighbors.append(i + 1)
        topology[i] = neighbors
    return topology


def create_full_topology(n: int) -> Dict[int, List[int]]:
    """Create fully connected topology: everyone talks to everyone."""
    return {i: [j for j in range(n) if j != i] for i in range(n)}
