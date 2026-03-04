#!/usr/bin/env python3
"""
Distributed Target Estimation via Gaussian Belief Propagation (GBP)

Problem Statement
-----------------
We consider n agents estimating a shared target position x ∈ ℝ². Each agent i
has a noisy observation z_i = x + ε_i where ε_i ~ N(0, Σ_obs). Agents can only
communicate with neighbors defined by a graph topology.

The goal is for all agents to converge to a consensus estimate x̂ that
approximates the centralized solution: x̂_central = (Σ Λ_i)^{-1} (Σ Λ_i z_i)
where Λ_i = Σ_obs^{-1} is the observation precision.

Factor Graph Formulation
------------------------
Each agent maintains a local factor graph with:
- Variable node: x (shared target position)
- Observation factor: f_i(x) ∝ exp(-0.5 ||z_i - x||²_{Λ_i})

For inter-agent communication, agents exchange their current belief on x.
GBP messages are Gaussian distributions in information form (η, Λ).

Algorithm (Synchronous GBP)
---------------------------
Initialize: b_i(x) = N(z_i, Σ_obs)  # Prior from observation

For each iteration:
    1. Each agent computes outgoing message to each neighbor:
       m_{i→j}(x) = b_i(x) / m_{j→i}(x)  # Cavity distribution

    2. Exchange messages with neighbors (via communication backend)

    3. Each agent updates belief:
       b_i(x) ∝ f_i(x) × Π_{j∈N(i)} m_{j→i}(x)

Convergence: When ||b_i - b_j|| < ε for all neighbors (i,j)

Usage
-----
    python gbp_distributed_estimation.py [--agents N] [--topology {ring,line,full}]

Output
------
- Convergence plot showing beliefs over iterations
- Final consensus vs centralized solution comparison
- Message passing statistics

This demo validates the communication backend and GBP implementation before
integrating with MuJoCo for control tasks.
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional
import argparse
import sys
import os

# Add parent to path for imports
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from communication.backend import (
    SimulatedBackend,
    GaussianMessage,
    create_ring_topology,
    create_line_topology,
    create_full_topology,
)


@dataclass
class GBPAgent:
    """
    Agent running local GBP for distributed estimation.

    Maintains:
    - Local observation and its precision
    - Current belief on shared variable x
    - Incoming messages from neighbors (through consensus factors)
    - Consensus factor precision λ

    Factor graph structure (per agent i):
        x_i ---- f_i (observation factor)
         |
        g_{ij} (consensus factor to each neighbor j)
         |
        x_j (neighbor's variable)

    Messages:
    - m_{x_i → g_{ij}}: cavity belief (belief excluding g_{ij}'s contribution)
    - m_{g_{ij} → x_i}: message from consensus factor (computed from neighbor's cavity)
    """
    agent_id: int
    observation: np.ndarray           # z_i ∈ ℝ²
    obs_precision: np.ndarray         # Λ_i = Σ_obs^{-1}
    neighbors: List[int] = field(default_factory=list)
    consensus_precision: float = 10.0  # λ for consensus factor g_{ij}

    # Current belief b_i(x) in information form
    belief_eta: np.ndarray = field(default=None)
    belief_lam: np.ndarray = field(default=None)

    # Messages FROM consensus factors: msg_from_factor[j] = m_{g_{ij} → x_i}
    messages_from_factor: Dict[int, GaussianMessage] = field(default_factory=dict)

    def __post_init__(self):
        dim = len(self.observation)
        # Initialize belief from observation only
        self.belief_lam = self.obs_precision.copy()
        self.belief_eta = self.obs_precision @ self.observation

        # Initialize incoming factor messages as uninformative (zero precision)
        zero_msg = GaussianMessage(
            eta=np.zeros(dim),
            lam=np.zeros((dim, dim))
        )
        for j in self.neighbors:
            self.messages_from_factor[j] = zero_msg.copy()

    @property
    def belief(self) -> GaussianMessage:
        """Current belief as GaussianMessage."""
        return GaussianMessage(eta=self.belief_eta.copy(), lam=self.belief_lam.copy())

    @property
    def belief_mean(self) -> np.ndarray:
        """Mean of current belief."""
        return np.linalg.solve(self.belief_lam, self.belief_eta)

    @property
    def belief_cov(self) -> np.ndarray:
        """Covariance of current belief."""
        return np.linalg.inv(self.belief_lam)

    def compute_cavity_to_factor(self, to_neighbor: int) -> GaussianMessage:
        """
        Compute message from variable x_i to consensus factor g_{ij}.

        m_{x_i → g_{ij}} = b_i(x) / m_{g_{ij} → x_i}

        This is the "cavity" distribution: belief excluding this factor's contribution.
        In information form: subtract the incoming factor message.
        """
        incoming = self.messages_from_factor[to_neighbor]

        # Cavity = belief minus incoming message from this factor
        eta_cavity = self.belief_eta - incoming.eta
        lam_cavity = self.belief_lam - incoming.lam

        # Ensure positive semi-definite (numerical safety)
        lam_cavity = 0.5 * (lam_cavity + lam_cavity.T)
        eigvals = np.linalg.eigvalsh(lam_cavity)
        if np.min(eigvals) < 1e-8:
            # Add small regularization
            lam_cavity += (abs(np.min(eigvals)) + 1e-6) * np.eye(len(eta_cavity))

        return GaussianMessage(eta=eta_cavity, lam=lam_cavity)

    def compute_factor_to_variable_message(
        self,
        neighbor_cavity: GaussianMessage
    ) -> GaussianMessage:
        """
        Compute message from consensus factor g_{ij} to variable x_i.

        m_{g_{ij} → x_i}(x_i) = ∫ g_{ij}(x_i, x_j) m_{x_j → g_{ij}}(x_j) dx_j

        For g_{ij}(x_i, x_j) = N(x_i - x_j | 0, λ^{-1}I):

        This integral convolves the neighbor's cavity belief with the consensus factor,
        yielding a Gaussian with:
            mean_out = mean_neighbor
            cov_out = cov_neighbor + λ^{-1}I

        Args:
            neighbor_cavity: Message from neighbor's variable to the shared factor
                            m_{x_j → g_{ij}} in information form (η_j, Λ_j)

        Returns:
            Message m_{g_{ij} → x_i} in information form
        """
        dim = len(neighbor_cavity.eta)
        lambda_I = self.consensus_precision * np.eye(dim)

        # Convert neighbor's cavity to moment form
        # Handle near-zero precision (uninformative message)
        if np.linalg.det(neighbor_cavity.lam) < 1e-10:
            # Neighbor has no information yet, return weak message
            return GaussianMessage(
                eta=np.zeros(dim),
                lam=np.zeros((dim, dim))
            )

        neighbor_cov = np.linalg.inv(neighbor_cavity.lam)
        neighbor_mean = neighbor_cov @ neighbor_cavity.eta

        # Convolve with consensus factor: cov_out = cov_neighbor + λ^{-1}I
        cov_out = neighbor_cov + (1.0 / self.consensus_precision) * np.eye(dim)

        # Mean passes through unchanged
        mean_out = neighbor_mean

        # Convert back to information form
        lam_out = np.linalg.inv(cov_out)
        eta_out = lam_out @ mean_out

        return GaussianMessage(eta=eta_out, lam=lam_out)

    def receive_factor_message(self, from_neighbor: int, message: GaussianMessage) -> None:
        """Store incoming message from consensus factor with neighbor."""
        self.messages_from_factor[from_neighbor] = message

    def update_belief(self) -> None:
        """
        Update belief by combining observation factor with consensus factor messages.

        b_i(x) ∝ f_i(x) × Π_{j∈N(i)} m_{g_{ij} → x_i}(x)

        In information form: sum observation contribution and all factor messages.
        """
        # Start with observation factor contribution
        self.belief_eta = self.obs_precision @ self.observation
        self.belief_lam = self.obs_precision.copy()

        # Add all incoming messages from consensus factors
        for j, msg in self.messages_from_factor.items():
            self.belief_eta = self.belief_eta + msg.eta
            self.belief_lam = self.belief_lam + msg.lam


class DistributedGBPEstimation:
    """
    Distributed estimation system using Gaussian Belief Propagation.

    Coordinates multiple GBP agents to estimate a shared variable.
    """

    def __init__(
        self,
        num_agents: int,
        target_true: np.ndarray,
        obs_noise_std: float = 0.5,
        consensus_precision: float = 10.0,
        topology: str = 'ring',
        backend=None,
        seed: int = None,
    ):
        """
        Initialize distributed estimation problem.

        Args:
            num_agents: Number of agents
            target_true: True target position [x, y]
            obs_noise_std: Observation noise standard deviation
            consensus_precision: λ for consensus factors g_{ij} (higher = stronger)
            topology: Communication topology ('ring', 'line', 'full').
                      Ignored if backend is provided.
            backend: Pre-constructed CommunicationBackend. If None, creates a
                     SimulatedBackend with the given topology string.
            seed: Random seed for reproducibility
        """
        if seed is not None:
            np.random.seed(seed)

        self.num_agents = num_agents
        self.target_true = np.array(target_true)
        self.obs_noise_std = obs_noise_std
        self.consensus_precision = consensus_precision
        self.dim = len(target_true)

        # Use injected backend, or create SimulatedBackend from topology string
        if backend is not None:
            self.backend = backend
            topo = backend.topology
            self.topology_name = type(backend).__name__
        else:
            if topology == 'ring':
                topo = create_ring_topology(num_agents)
            elif topology == 'line':
                topo = create_line_topology(num_agents)
            elif topology == 'full':
                topo = create_full_topology(num_agents)
            else:
                raise ValueError(f"Unknown topology: {topology}")
            self.topology_name = topology
            self.backend = SimulatedBackend(num_agents, topo)

        # Observation precision (same for all agents)
        obs_cov = (obs_noise_std ** 2) * np.eye(self.dim)
        obs_precision = np.linalg.inv(obs_cov)

        # Generate noisy observations and create agents
        self.agents: List[GBPAgent] = []
        self.observations = []

        for i in range(num_agents):
            noise = np.random.randn(self.dim) * obs_noise_std
            obs = self.target_true + noise
            self.observations.append(obs)

            agent = GBPAgent(
                agent_id=i,
                observation=obs,
                obs_precision=obs_precision,
                neighbors=topo[i],
                consensus_precision=consensus_precision,
            )
            self.agents.append(agent)

        # Compute centralized solution for comparison
        self.centralized_solution = self._compute_centralized()

        # History for plotting
        self.belief_history: List[List[np.ndarray]] = []
        self.consensus_error_history: List[float] = []

    def _compute_centralized(self) -> np.ndarray:
        """
        Compute centralized fusion of all observations.

        x̂ = (Σ Λ_i)^{-1} (Σ Λ_i z_i) = mean(z_i) for uniform precision
        """
        return np.mean(self.observations, axis=0)

    def run_iteration(self) -> float:
        """
        Run one iteration of synchronous GBP with explicit consensus factors.

        Message passing flow:
        1. Each agent computes cavity message m_{x_i → g_{ij}} for each neighbor j
        2. Cavity messages are sent to neighbors (these will be used to compute factor messages)
        3. Barrier (synchronize)
        4. Each agent receives neighbor's cavity messages
        5. Each agent computes factor messages m_{g_{ij} → x_i} from received cavities
        6. Each agent updates belief

        Returns:
            Consensus error (max disagreement between neighbors)
        """
        # 1. Compute cavity messages and send to neighbors
        #    Agent i sends m_{x_i → g_{ij}} to agent j
        #    Agent j will use this to compute m_{g_{ij} → x_j}
        for agent in self.agents:
            for neighbor in agent.neighbors:
                cavity_msg = agent.compute_cavity_to_factor(neighbor)
                self.backend.send(agent.agent_id, neighbor, cavity_msg)

        # 2. Synchronization barrier
        self.backend.barrier()

        # 3. Receive neighbor's cavity messages and compute factor-to-variable messages
        for agent in self.agents:
            inbox = self.backend.receive(agent.agent_id)
            for sender_id, neighbor_cavity in inbox:
                # Compute message from consensus factor g_{sender,agent} to agent
                # This uses neighbor's cavity and passes it through the consensus factor
                factor_msg = agent.compute_factor_to_variable_message(neighbor_cavity)
                agent.receive_factor_message(sender_id, factor_msg)

        # 4. Update beliefs
        for agent in self.agents:
            agent.update_belief()

        # 5. Record history and compute consensus error
        beliefs = [agent.belief_mean.copy() for agent in self.agents]
        self.belief_history.append(beliefs)

        # Consensus error: max distance between any two agents
        max_disagreement = 0.0
        for i, agent in enumerate(self.agents):
            for j in agent.neighbors:
                dist = np.linalg.norm(beliefs[i] - beliefs[j])
                max_disagreement = max(max_disagreement, dist)

        self.consensus_error_history.append(max_disagreement)
        return max_disagreement

    def run(self, max_iterations: int = 50, tol: float = 1e-4) -> Dict:
        """
        Run GBP until convergence or max iterations.

        Returns:
            Dict with results and statistics
        """
        # Record initial beliefs
        initial_beliefs = [agent.belief_mean.copy() for agent in self.agents]
        self.belief_history.append(initial_beliefs)

        converged = False
        for iteration in range(max_iterations):
            error = self.run_iteration()

            if error < tol:
                converged = True
                break

        # Final results
        final_beliefs = [agent.belief_mean for agent in self.agents]
        consensus_mean = np.mean(final_beliefs, axis=0)

        # Error vs centralized solution
        error_vs_central = np.linalg.norm(consensus_mean - self.centralized_solution)

        # Error vs true target
        error_vs_true = np.linalg.norm(consensus_mean - self.target_true)

        results = {
            'converged': converged,
            'iterations': len(self.consensus_error_history),
            'final_consensus_error': self.consensus_error_history[-1],
            'consensus_mean': consensus_mean,
            'centralized_solution': self.centralized_solution,
            'target_true': self.target_true,
            'error_vs_centralized': error_vs_central,
            'error_vs_true': error_vs_true,
            'communication_stats': self.backend.get_stats(),
            'num_agents': self.num_agents,
            'topology': self.topology_name,
            'observations': np.array(self.observations),
        }

        return results

    def plot_results(self, save_path: str = None, show: bool = True) -> None:
        """Generate visualization of GBP convergence."""
        fig, axes = plt.subplots(1, 3, figsize=(14, 4))

        # Plot 1: Agent beliefs over iterations (2D trajectory)
        ax1 = axes[0]
        colors = plt.cm.viridis(np.linspace(0, 1, self.num_agents))

        for i in range(self.num_agents):
            traj = np.array([beliefs[i] for beliefs in self.belief_history])
            ax1.plot(traj[:, 0], traj[:, 1], '-', color=colors[i], alpha=0.5, linewidth=1)
            ax1.scatter(traj[0, 0], traj[0, 1], color=colors[i], s=50, marker='o',
                       edgecolors='black', label=f'Agent {i}' if i < 4 else None)
            ax1.scatter(traj[-1, 0], traj[-1, 1], color=colors[i], s=100, marker='s',
                       edgecolors='black')

        # Mark true target and centralized solution
        ax1.scatter(*self.target_true, color='red', s=200, marker='*',
                   label='True target', zorder=10)
        ax1.scatter(*self.centralized_solution, color='green', s=150, marker='^',
                   label='Centralized', zorder=10)

        ax1.set_xlabel('x')
        ax1.set_ylabel('y')
        ax1.set_title('Belief Trajectories\n(○ initial → □ final)')
        ax1.legend(loc='upper right', fontsize=8)
        ax1.set_aspect('equal')
        ax1.grid(True, alpha=0.3)

        # Plot 2: Consensus error over iterations
        ax2 = axes[1]
        iterations = range(len(self.consensus_error_history))
        ax2.semilogy(iterations, self.consensus_error_history, 'b-', linewidth=2)
        ax2.axhline(y=1e-4, color='r', linestyle='--', label='Tolerance')
        ax2.set_xlabel('Iteration')
        ax2.set_ylabel('Consensus Error (max disagreement)')
        ax2.set_title('GBP Convergence')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Plot 3: Final beliefs vs true
        ax3 = axes[2]
        final_beliefs = self.belief_history[-1]
        agent_ids = range(self.num_agents)

        # X and Y components
        final_x = [b[0] for b in final_beliefs]
        final_y = [b[1] for b in final_beliefs]

        ax3.bar(np.array(agent_ids) - 0.15, final_x, 0.3, label='x estimate', color='steelblue')
        ax3.bar(np.array(agent_ids) + 0.15, final_y, 0.3, label='y estimate', color='coral')

        ax3.axhline(y=self.target_true[0], color='steelblue', linestyle='--', alpha=0.5)
        ax3.axhline(y=self.target_true[1], color='coral', linestyle='--', alpha=0.5)

        ax3.set_xlabel('Agent ID')
        ax3.set_ylabel('Estimate')
        ax3.set_title('Final Beliefs\n(dashed = true)')
        ax3.legend()
        ax3.set_xticks(agent_ids)
        ax3.grid(True, alpha=0.3, axis='y')

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Figure saved to: {save_path}")

        if show:
            plt.show()


def run_scaling_experiment(
    agent_counts: List[int] = [2, 4, 8, 16, 32],
    topology: str = 'ring',
    trials: int = 5,
) -> Dict:
    """
    Run scaling experiment: how does convergence scale with n?

    Returns dict with iterations and messages vs agent count.
    """
    results = {
        'agent_counts': agent_counts,
        'mean_iterations': [],
        'std_iterations': [],
        'mean_messages': [],
        'topology': topology,
    }

    for n in agent_counts:
        trial_iters = []
        trial_msgs = []

        for trial in range(trials):
            system = DistributedGBPEstimation(
                num_agents=n,
                target_true=np.array([5.0, 3.0]),
                obs_noise_std=0.5,
                topology=topology,
                seed=42 + trial,
            )
            result = system.run(max_iterations=200)
            trial_iters.append(result['iterations'])
            trial_msgs.append(result['communication_stats']['messages_sent'])

        results['mean_iterations'].append(np.mean(trial_iters))
        results['std_iterations'].append(np.std(trial_iters))
        results['mean_messages'].append(np.mean(trial_msgs))

    return results


def main():
    parser = argparse.ArgumentParser(
        description='Distributed Target Estimation via GBP',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python gbp_distributed_estimation.py                    # Default: 4 agents, ring
  python gbp_distributed_estimation.py -n 8 -t full       # 8 agents, fully connected
  python gbp_distributed_estimation.py --scaling          # Run scaling experiment
        """
    )
    parser.add_argument('-n', '--agents', type=int, default=4,
                       help='Number of agents (default: 4)')
    parser.add_argument('-t', '--topology', choices=['ring', 'line', 'full'],
                       default='ring', help='Communication topology (default: ring)')
    parser.add_argument('--noise', type=float, default=0.5,
                       help='Observation noise std (default: 0.5)')
    parser.add_argument('--lambda', dest='consensus_precision', type=float, default=10.0,
                       help='Consensus factor precision λ (default: 10.0)')
    parser.add_argument('--seed', type=int, default=42,
                       help='Random seed (default: 42)')
    parser.add_argument('--max-iter', type=int, default=50,
                       help='Max GBP iterations (default: 50)')
    parser.add_argument('--scaling', action='store_true',
                       help='Run scaling experiment instead of single demo')
    parser.add_argument('--save', type=str, default=None,
                       help='Save figure to path')
    parser.add_argument('--no-plot', action='store_true',
                       help='Skip plotting (for automated testing)')

    args = parser.parse_args()

    if args.scaling:
        # Run scaling experiment
        print("="*60)
        print("GBP Scaling Experiment")
        print("="*60)

        for topology in ['ring', 'line', 'full']:
            print(f"\nTopology: {topology}")
            results = run_scaling_experiment(
                agent_counts=[2, 4, 8, 16, 32],
                topology=topology,
            )

            print(f"  {'n':>4} | {'Iterations':>12} | {'Messages':>10}")
            print(f"  {'-'*4}-+-{'-'*12}-+-{'-'*10}")
            for i, n in enumerate(results['agent_counts']):
                iters = results['mean_iterations'][i]
                msgs = results['mean_messages'][i]
                print(f"  {n:4d} | {iters:12.1f} | {msgs:10.0f}")

        return

    # Single demo run
    print("="*60)
    print("Distributed Target Estimation via GBP")
    print("="*60)
    print(f"  Agents: {args.agents}")
    print(f"  Topology: {args.topology}")
    print(f"  Observation noise σ: {args.noise}")
    print(f"  Consensus precision λ: {args.consensus_precision}")
    print()

    # Create and run system
    system = DistributedGBPEstimation(
        num_agents=args.agents,
        target_true=np.array([5.0, 3.0]),
        obs_noise_std=args.noise,
        consensus_precision=args.consensus_precision,
        topology=args.topology,
        seed=args.seed,
    )

    print("Initial observations:")
    for i, obs in enumerate(system.observations):
        print(f"  Agent {i}: z = [{obs[0]:.3f}, {obs[1]:.3f}]")
    print(f"  True target: [{system.target_true[0]:.3f}, {system.target_true[1]:.3f}]")
    print()

    # Run GBP
    results = system.run(max_iterations=args.max_iter)

    # Print results
    print("Results:")
    print(f"  Converged: {results['converged']}")
    print(f"  Iterations: {results['iterations']}")
    print(f"  Final consensus error: {results['final_consensus_error']:.2e}")
    print()
    print(f"  Consensus estimate: [{results['consensus_mean'][0]:.4f}, {results['consensus_mean'][1]:.4f}]")
    print(f"  Centralized solution: [{results['centralized_solution'][0]:.4f}, {results['centralized_solution'][1]:.4f}]")
    print(f"  True target: [{results['target_true'][0]:.4f}, {results['target_true'][1]:.4f}]")
    print()
    print(f"  Error vs centralized: {results['error_vs_centralized']:.4e}")
    print(f"  Error vs true: {results['error_vs_true']:.4f}")
    print()
    print(f"Communication stats:")
    stats = results['communication_stats']
    print(f"  Messages sent: {stats['messages_sent']}")
    print(f"  Barrier calls: {stats['barrier_calls']}")

    # Plot
    if not args.no_plot:
        save_path = args.save
        if save_path is None:
            # Default save location
            fig_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                                   'analysis', 'figures')
            os.makedirs(fig_dir, exist_ok=True)
            save_path = os.path.join(fig_dir, 'gbp_demo.pdf')

        system.plot_results(save_path=save_path)


if __name__ == '__main__':
    main()
