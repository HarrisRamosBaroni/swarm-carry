#!/usr/bin/env python3
"""
Factor Graph Visualization for Distributed GBP

Visualizes:
1. Global factor graph (centralized view)
2. Distributed factor graph (with agent-local copies of shared variables)
3. Per-agent local subgraphs (what each agent actually maintains in memory)
4. Message passing edges (inter-agent communication)

This helps understand:
- Where internal GBP happens (within agent's local graph)
- Where inter-agent GBP happens (messages over network)
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
import argparse
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from communication.backend import (
    create_ring_topology,
    create_line_topology,
    create_full_topology,
)


@dataclass
class FactorNode:
    """A factor in the factor graph."""
    name: str
    connected_vars: List[str]
    owner: Optional[int] = None  # Which agent owns this factor (None = shared)
    factor_type: str = "observation"  # observation, dynamics, consensus, cost


@dataclass
class VariableNode:
    """A variable in the factor graph."""
    name: str
    owner: Optional[int] = None  # Which agent owns this variable
    is_shared: bool = False      # Is this a shared variable (e.g., payload state)?
    dim: int = 2


def build_centralized_graph(num_agents: int) -> Tuple[List[VariableNode], List[FactorNode]]:
    """
    Build the centralized factor graph for target estimation.

    Structure:
        x (target) --- f_i (observation) for each agent i

    This is what a centralized solver would see.
    """
    variables = [
        VariableNode(name="x", owner=None, is_shared=True, dim=2)
    ]

    factors = []
    for i in range(num_agents):
        factors.append(FactorNode(
            name=f"f_{i}",
            connected_vars=["x"],
            owner=i,
            factor_type="observation"
        ))

    return variables, factors


def build_distributed_graph(
    num_agents: int,
    topology: Dict[int, List[int]]
) -> Tuple[List[VariableNode], List[FactorNode]]:
    """
    Build the distributed factor graph for target estimation.

    Structure:
        Each agent i has:
        - x_i (local copy of shared variable x)
        - f_i (observation factor)

        Between neighboring agents i,j:
        - g_{ij} (consensus factor enforcing x_i ≈ x_j)

    This is how the problem is actually partitioned for distributed solving.
    """
    variables = []
    factors = []

    # Each agent has a local copy of the shared variable
    for i in range(num_agents):
        variables.append(VariableNode(
            name=f"x_{i}",
            owner=i,
            is_shared=True,  # It's a copy of the shared variable
            dim=2
        ))

    # Each agent has an observation factor
    for i in range(num_agents):
        factors.append(FactorNode(
            name=f"f_{i}",
            connected_vars=[f"x_{i}"],
            owner=i,
            factor_type="observation"
        ))

    # Consensus factors between neighbors (add once per edge)
    added_edges = set()
    for i in range(num_agents):
        for j in topology.get(i, []):
            edge = tuple(sorted([i, j]))
            if edge not in added_edges:
                factors.append(FactorNode(
                    name=f"g_{{{i},{j}}}",
                    connected_vars=[f"x_{i}", f"x_{j}"],
                    owner=None,  # Shared between agents
                    factor_type="consensus"
                ))
                added_edges.add(edge)

    return variables, factors


def get_agent_local_graph(
    agent_id: int,
    variables: List[VariableNode],
    factors: List[FactorNode],
    topology: Dict[int, List[int]]
) -> Tuple[List[VariableNode], List[FactorNode], List[str]]:
    """
    Extract the subgraph that agent_id maintains in memory.

    Returns:
        - Local variables (owned by this agent)
        - Local factors (owned by this agent + consensus factors with neighbors)
        - Inter-agent edges (variable names that require network communication)
    """
    local_vars = [v for v in variables if v.owner == agent_id]
    local_var_names = {v.name for v in local_vars}

    # Factors owned by this agent
    local_factors = [f for f in factors if f.owner == agent_id]

    # Consensus factors involving this agent's variables
    for f in factors:
        if f.factor_type == "consensus":
            if any(v in local_var_names for v in f.connected_vars):
                if f not in local_factors:
                    local_factors.append(f)

    # Inter-agent edges: neighbor's variables that appear in consensus factors
    inter_agent_vars = []
    for f in local_factors:
        if f.factor_type == "consensus":
            for v in f.connected_vars:
                if v not in local_var_names:
                    inter_agent_vars.append(v)

    return local_vars, local_factors, inter_agent_vars


def plot_centralized_graph(
    num_agents: int,
    ax: plt.Axes = None,
    title: str = "Centralized Factor Graph"
) -> plt.Axes:
    """Plot the centralized (global) factor graph."""
    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 8))

    variables, factors = build_centralized_graph(num_agents)

    # Position: x in center, factors around it
    center = np.array([0.5, 0.5])
    var_pos = {"x": center}

    angles = np.linspace(0, 2*np.pi, num_agents, endpoint=False) - np.pi/2
    radius = 0.35
    factor_pos = {}
    for i, f in enumerate(factors):
        factor_pos[f.name] = center + radius * np.array([np.cos(angles[i]), np.sin(angles[i])])

    # Draw edges
    for f in factors:
        for v in f.connected_vars:
            ax.plot(
                [var_pos[v][0], factor_pos[f.name][0]],
                [var_pos[v][1], factor_pos[f.name][1]],
                'k-', linewidth=1.5, zorder=1
            )

    # Draw variable node (circle)
    ax.scatter(*var_pos["x"], s=800, c='lightblue', edgecolors='black',
               linewidths=2, zorder=3, marker='o')
    ax.annotate("x", var_pos["x"], ha='center', va='center', fontsize=14, fontweight='bold')

    # Draw factor nodes (squares)
    colors = plt.cm.Set3(np.linspace(0, 1, num_agents))
    for i, f in enumerate(factors):
        ax.scatter(*factor_pos[f.name], s=600, c=[colors[i]], edgecolors='black',
                   linewidths=2, zorder=3, marker='s')
        ax.annotate(f.name, factor_pos[f.name], ha='center', va='center', fontsize=10)

    ax.set_xlim(-0.1, 1.1)
    ax.set_ylim(-0.1, 1.1)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title(title, fontsize=12, fontweight='bold')

    return ax


def plot_distributed_graph(
    num_agents: int,
    topology: Dict[int, List[int]],
    ax: plt.Axes = None,
    title: str = "Distributed Factor Graph"
) -> plt.Axes:
    """Plot the distributed factor graph with consensus factors."""
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 10))

    variables, factors = build_distributed_graph(num_agents, topology)

    # Position agents in a circle
    angles = np.linspace(0, 2*np.pi, num_agents, endpoint=False) - np.pi/2
    radius = 0.35
    center = np.array([0.5, 0.5])

    # Variable positions (around center)
    var_pos = {}
    for i in range(num_agents):
        var_pos[f"x_{i}"] = center + radius * np.array([np.cos(angles[i]), np.sin(angles[i])])

    # Factor positions (slightly outside variables)
    obs_factor_pos = {}
    for i in range(num_agents):
        obs_factor_pos[f"f_{i}"] = center + (radius + 0.12) * np.array([np.cos(angles[i]), np.sin(angles[i])])

    # Consensus factor positions (midpoint between connected variables)
    consensus_factor_pos = {}
    for f in factors:
        if f.factor_type == "consensus":
            v1, v2 = f.connected_vars
            consensus_factor_pos[f.name] = 0.5 * (var_pos[v1] + var_pos[v2])

    # Colors for agents
    colors = plt.cm.Set2(np.linspace(0, 1, num_agents))

    # Draw edges: observation factors to variables
    for f in factors:
        if f.factor_type == "observation":
            v = f.connected_vars[0]
            ax.plot(
                [var_pos[v][0], obs_factor_pos[f.name][0]],
                [var_pos[v][1], obs_factor_pos[f.name][1]],
                '-', color=colors[f.owner], linewidth=2, zorder=1
            )

    # Draw edges: consensus factors (inter-agent communication)
    for f in factors:
        if f.factor_type == "consensus":
            v1, v2 = f.connected_vars
            # Draw dashed line to show this is inter-agent
            ax.plot(
                [var_pos[v1][0], consensus_factor_pos[f.name][0]],
                [var_pos[v1][1], consensus_factor_pos[f.name][1]],
                '--', color='red', linewidth=2, zorder=1, alpha=0.7
            )
            ax.plot(
                [var_pos[v2][0], consensus_factor_pos[f.name][0]],
                [var_pos[v2][1], consensus_factor_pos[f.name][1]],
                '--', color='red', linewidth=2, zorder=1, alpha=0.7
            )

    # Draw variable nodes
    for i in range(num_agents):
        name = f"x_{i}"
        ax.scatter(*var_pos[name], s=700, c=[colors[i]], edgecolors='black',
                   linewidths=2, zorder=3, marker='o')
        ax.annotate(name, var_pos[name], ha='center', va='center', fontsize=11, fontweight='bold')

    # Draw observation factor nodes
    for i in range(num_agents):
        name = f"f_{i}"
        ax.scatter(*obs_factor_pos[name], s=500, c=[colors[i]], edgecolors='black',
                   linewidths=2, zorder=3, marker='s')
        ax.annotate(name, obs_factor_pos[name], ha='center', va='center', fontsize=9)

    # Draw consensus factor nodes
    for f in factors:
        if f.factor_type == "consensus":
            ax.scatter(*consensus_factor_pos[f.name], s=400, c='salmon', edgecolors='darkred',
                       linewidths=2, zorder=3, marker='D')
            ax.annotate(f.name, consensus_factor_pos[f.name], ha='center', va='center', fontsize=8)

    # Legend
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor='lightblue',
               markersize=12, markeredgecolor='black', label='Variable node'),
        Line2D([0], [0], marker='s', color='w', markerfacecolor='lightgreen',
               markersize=10, markeredgecolor='black', label='Observation factor'),
        Line2D([0], [0], marker='D', color='w', markerfacecolor='salmon',
               markersize=10, markeredgecolor='darkred', label='Consensus factor'),
        Line2D([0], [0], color='gray', linewidth=2, linestyle='-', label='Local edge'),
        Line2D([0], [0], color='red', linewidth=2, linestyle='--', label='Inter-agent edge'),
    ]
    ax.legend(handles=legend_elements, loc='upper left', fontsize=8)

    ax.set_xlim(-0.05, 1.05)
    ax.set_ylim(-0.05, 1.05)
    ax.set_aspect('equal')
    ax.axis('off')
    ax.set_title(title, fontsize=12, fontweight='bold')

    return ax


def plot_agent_local_graphs(
    num_agents: int,
    topology: Dict[int, List[int]],
    figsize: Tuple[int, int] = None
) -> plt.Figure:
    """
    Plot what each agent maintains in memory (local subgraph).

    Shows:
    - Variables owned by the agent
    - Factors owned by the agent
    - Edges to neighbor variables (grayed out, representing inter-agent messages)
    """
    variables, factors = build_distributed_graph(num_agents, topology)

    # Determine grid layout
    cols = min(num_agents, 4)
    rows = (num_agents + cols - 1) // cols

    if figsize is None:
        figsize = (4 * cols, 4 * rows)

    fig, axes = plt.subplots(rows, cols, figsize=figsize)
    if num_agents == 1:
        axes = np.array([[axes]])
    elif rows == 1:
        axes = axes.reshape(1, -1)
    elif cols == 1:
        axes = axes.reshape(-1, 1)

    colors = plt.cm.Set2(np.linspace(0, 1, num_agents))

    for agent_id in range(num_agents):
        row, col = agent_id // cols, agent_id % cols
        ax = axes[row, col]

        local_vars, local_factors, inter_agent_vars = get_agent_local_graph(
            agent_id, variables, factors, topology
        )

        # Layout: local variable in center, factors and neighbor vars around
        center = np.array([0.5, 0.5])

        # Position local variable
        local_var_pos = {local_vars[0].name: center}

        # Position observation factor above
        obs_factor = [f for f in local_factors if f.factor_type == "observation"][0]
        obs_pos = {obs_factor.name: center + np.array([0, 0.25])}

        # Position consensus factors and neighbor variables around
        consensus_factors = [f for f in local_factors if f.factor_type == "consensus"]
        n_consensus = len(consensus_factors)

        if n_consensus > 0:
            angles = np.linspace(-np.pi/2, np.pi/2, n_consensus + 2)[1:-1] + np.pi
            consensus_pos = {}
            neighbor_var_pos = {}

            for idx, f in enumerate(consensus_factors):
                angle = angles[idx]
                consensus_pos[f.name] = center + 0.2 * np.array([np.cos(angle), np.sin(angle)])

                # Find neighbor variable
                for v in f.connected_vars:
                    if v not in local_var_pos:
                        neighbor_var_pos[v] = center + 0.35 * np.array([np.cos(angle), np.sin(angle)])

        # Draw edges
        # Local variable to observation factor
        ax.plot(
            [local_var_pos[local_vars[0].name][0], obs_pos[obs_factor.name][0]],
            [local_var_pos[local_vars[0].name][1], obs_pos[obs_factor.name][1]],
            '-', color=colors[agent_id], linewidth=2.5, zorder=1
        )

        # Consensus edges
        for f in consensus_factors:
            pos_f = consensus_pos[f.name]
            # To local var
            ax.plot(
                [local_var_pos[local_vars[0].name][0], pos_f[0]],
                [local_var_pos[local_vars[0].name][1], pos_f[1]],
                '-', color=colors[agent_id], linewidth=2, zorder=1
            )
            # To neighbor var (dashed = network)
            for v in f.connected_vars:
                if v in neighbor_var_pos:
                    ax.plot(
                        [pos_f[0], neighbor_var_pos[v][0]],
                        [pos_f[1], neighbor_var_pos[v][1]],
                        '--', color='red', linewidth=2, zorder=1, alpha=0.7
                    )

        # Draw nodes
        # Local variable
        ax.scatter(*local_var_pos[local_vars[0].name], s=600, c=[colors[agent_id]],
                   edgecolors='black', linewidths=2, zorder=3, marker='o')
        ax.annotate(local_vars[0].name, local_var_pos[local_vars[0].name],
                    ha='center', va='center', fontsize=11, fontweight='bold')

        # Observation factor
        ax.scatter(*obs_pos[obs_factor.name], s=450, c=[colors[agent_id]],
                   edgecolors='black', linewidths=2, zorder=3, marker='s')
        ax.annotate(obs_factor.name, obs_pos[obs_factor.name],
                    ha='center', va='center', fontsize=10)

        # Consensus factors
        for f in consensus_factors:
            ax.scatter(*consensus_pos[f.name], s=350, c='salmon', edgecolors='darkred',
                       linewidths=2, zorder=3, marker='D')
            # Shorter label
            short_label = f.name.replace("{", "").replace("}", "").replace(",", "")
            ax.annotate(f"g{short_label[1:]}", consensus_pos[f.name],
                        ha='center', va='center', fontsize=8)

        # Neighbor variables (grayed out - not stored locally, just received via messages)
        for v, pos in neighbor_var_pos.items():
            ax.scatter(*pos, s=400, c='lightgray', edgecolors='gray',
                       linewidths=2, zorder=2, marker='o', alpha=0.6)
            ax.annotate(v, pos, ha='center', va='center', fontsize=9, color='gray')

        ax.set_xlim(0, 1)
        ax.set_ylim(0.1, 0.9)
        ax.set_aspect('equal')
        ax.axis('off')
        ax.set_title(f"Agent {agent_id}'s Local Graph", fontsize=11, fontweight='bold',
                     color=colors[agent_id])

        # Add box around agent's "memory"
        rect = mpatches.FancyBboxPatch(
            (0.05, 0.15), 0.9, 0.7,
            boxstyle="round,pad=0.02,rounding_size=0.05",
            facecolor=colors[agent_id], alpha=0.1,
            edgecolor=colors[agent_id], linewidth=2
        )
        ax.add_patch(rect)

    # Hide unused subplots
    for idx in range(num_agents, rows * cols):
        row, col = idx // cols, idx % cols
        axes[row, col].axis('off')

    # Add global legend
    legend_elements = [
        Line2D([0], [0], marker='o', color='w', markerfacecolor='lightblue',
               markersize=10, markeredgecolor='black', label='Local variable (in memory)'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor='lightgray',
               markersize=10, markeredgecolor='gray', label='Neighbor variable (via message)'),
        Line2D([0], [0], marker='s', color='w', markerfacecolor='lightgreen',
               markersize=9, markeredgecolor='black', label='Observation factor'),
        Line2D([0], [0], marker='D', color='w', markerfacecolor='salmon',
               markersize=9, markeredgecolor='darkred', label='Consensus factor'),
        Line2D([0], [0], color='green', linewidth=2, linestyle='-', label='Internal GBP'),
        Line2D([0], [0], color='red', linewidth=2, linestyle='--', label='Inter-agent GBP'),
    ]
    fig.legend(handles=legend_elements, loc='lower center', ncol=3, fontsize=9,
               bbox_to_anchor=(0.5, 0.02))

    plt.tight_layout(rect=[0, 0.08, 1, 1])

    return fig


def create_full_visualization(
    num_agents: int,
    topology_name: str = 'line',
    save_path: str = None,
    show: bool = True
):
    """Create complete visualization with all views."""

    # Get topology
    if topology_name == 'ring':
        topology = create_ring_topology(num_agents)
    elif topology_name == 'line':
        topology = create_line_topology(num_agents)
    elif topology_name == 'full':
        topology = create_full_topology(num_agents)
    else:
        raise ValueError(f"Unknown topology: {topology_name}")

    # Figure 1: Centralized vs Distributed comparison
    fig1, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    plot_centralized_graph(num_agents, ax1, "Centralized Factor Graph\n(what a global solver sees)")
    plot_distributed_graph(num_agents, topology, ax2,
                          f"Distributed Factor Graph ({topology_name} topology)\n(with consensus factors)")

    plt.tight_layout()

    if save_path:
        base, ext = os.path.splitext(save_path)
        path1 = f"{base}_global{ext}"
        fig1.savefig(path1, dpi=150, bbox_inches='tight')
        print(f"Saved: {path1}")

    # Figure 2: Per-agent local graphs
    fig2 = plot_agent_local_graphs(num_agents, topology)
    fig2.suptitle(f"Per-Agent Local Subgraphs ({topology_name} topology, n={num_agents})",
                  fontsize=14, fontweight='bold', y=1.02)

    if save_path:
        path2 = f"{base}_local{ext}"
        fig2.savefig(path2, dpi=150, bbox_inches='tight')
        print(f"Saved: {path2}")

    if show:
        plt.show()

    return fig1, fig2


def main():
    parser = argparse.ArgumentParser(
        description='Visualize factor graphs for distributed GBP',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python visualize_factor_graph.py                     # 4 agents, line topology
  python visualize_factor_graph.py -n 6 -t ring       # 6 agents, ring topology
  python visualize_factor_graph.py -n 3 -t full       # 3 agents, fully connected
        """
    )
    parser.add_argument('-n', '--agents', type=int, default=4,
                       help='Number of agents (default: 4)')
    parser.add_argument('-t', '--topology', choices=['ring', 'line', 'full'],
                       default='line', help='Communication topology (default: line)')
    parser.add_argument('--save', type=str, default=None,
                       help='Save figures to path (adds _global.pdf and _local.pdf)')
    parser.add_argument('--no-show', action='store_true',
                       help='Do not display figures')

    args = parser.parse_args()

    print("="*60)
    print("Factor Graph Visualization for Distributed GBP")
    print("="*60)
    print(f"  Agents: {args.agents}")
    print(f"  Topology: {args.topology}")
    print()

    # Default save path
    save_path = args.save
    if save_path is None:
        fig_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)),
                               'analysis', 'figures')
        os.makedirs(fig_dir, exist_ok=True)
        save_path = os.path.join(fig_dir, 'factor_graph.pdf')

    create_full_visualization(
        num_agents=args.agents,
        topology_name=args.topology,
        save_path=save_path,
        show=not args.no_show
    )


if __name__ == '__main__':
    main()
