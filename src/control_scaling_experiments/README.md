# Control Scaling Experiments for Multi-Robot Payload Transport

This package implements computational scaling experiments for multi-robot payload transport tasks. Test different control algorithms (centralized, distributed, MPC, optimization-based, etc.) and measure how their computational cost scales with team size.

## Overview

**Research Question:** How does the computational cost of different control algorithms scale with team size?

**Example (Centralized MPC):** How does MPC solve time scale with the number of robots?

**Hypothesis:** The solve time τ follows a power law: `τ = α n^β`

where:
- `n` = number of robots
- `α` = solver-dependent constant
- `β` = scaling exponent (typically between 1.5 and 3.0)

**Experimental Setup:**
- n robots push a long cuboid along one face
- Robots arranged symmetrically (no rotation)
- Payload must travel a fixed distance (default: 10m)
- MPC controller computes velocity commands

## Package Structure

```
control_scaling_experiments/
├── controllers/
│   ├── base_controller.py      # Abstract controller interface
│   └── centralized_mpc.py      # MPC implementation (CasADi + IPOPT)
├── scenarios/
│   ├── generate_mpc_scene.py   # Scene generator (dynamic cuboid sizing)
│   └── scenes/                 # Generated MuJoCo XML files
├── experiments/
│   └── run_scaling_experiment.py  # Experiment runner with logging
├── analysis/
│   ├── plot_scaling_laws.py    # Plot generator
│   ├── logs/                   # Experiment logs (JSON)
│   └── figures/                # Generated figures (PDF)
├── requirements.txt
└── README.md
```

## Installation

### Prerequisites

- Python 3.8+
- MuJoCo (for simulation)

### Setup

```bash
cd src/swarm_mpc_experiments
pip install -r requirements.txt
```

**Note:** CasADi (MPC solver library) will be installed automatically.

## Usage

### 1. Run Scaling Experiments

**Basic usage** (test n=2,4,8,16):
```bash
cd experiments
python run_scaling_experiment.py
```

**Custom n values:**
```bash
python run_scaling_experiment.py -n 2,4,8,16,32,64,128
```

**With visualization** (slower, good for debugging):
```bash
python run_scaling_experiment.py -n 2,4,8 --viewer
```

**Full options:**
```bash
python run_scaling_experiment.py \
  -n 2,4,8,16,32,64 \        # Robot counts to test
  -d 10.0 \                  # Push distance (meters)
  -t 60.0 \                  # Max simulation time per run
  --horizon 20 \             # MPC prediction horizon
  --dt 0.05 \                # MPC time step
  --solver ipopt \           # Solver (ipopt, sqpmethod, etc.)
  -o ../analysis/logs        # Output directory
```

**Expected output:**
```
============================================================
Experiment: n=4 robots
============================================================
MPC scene generated: scenarios/scenes/mpc_scene_n4.xml
  - Robots: 4
  - Cuboid dimensions: 0.30 × 1.40 × 0.40 m
  - Cuboid mass: 84.0 kg
MPC Problem Built:
  - Horizon: 20 steps (1.00s)
  - Decision variables: 160
  - Constraints: 80
  - Solver: ipopt
Running experiment (n=4)...
  t=10.0s | d=5.23m | solve_time=18.3ms
  ✓ Completed: converged=True, final_dist=0.12m, mean_solve=19.1ms

Results saved: ../analysis/logs/mpc_scaling_20240102_143022.json
```

### 2. Generate Plots

**After running experiments:**
```bash
cd ../analysis
python plot_scaling_laws.py
```

**Custom log file:**
```bash
python plot_scaling_laws.py --log logs/mpc_scaling_20240102_143022.json
```

**Compare multiple runs** (e.g., different solvers):
```bash
python plot_scaling_laws.py \
  --log ipopt_run.json,acados_run.json \
  --labels "IPOPT,acados"
```

**Generated figures:**
- `solve_time_vs_n.pdf` - **Primary result:** scaling law with power law fit
- `performance_metrics.pdf` - Convergence time and accuracy vs n
- `trajectories.pdf` - Example payload paths for different n
- `contacts_vs_n.pdf` - Contact dynamics analysis
- `summary.txt` - Text summary of all runs

### 3. Add New Controllers

To implement a distributed or alternative controller:

```python
# controllers/my_controller.py
from controllers.base_controller import BaseController
import numpy as np

class MyController(BaseController):
    def __init__(self, num_robots, config=None):
        super().__init__(num_robots, config)
        # Your initialization

    def compute_control(self, payload_state, robot_states, goal_state, dt):
        # Your control logic
        controls = np.zeros((self.num_robots, 2))
        # ... compute controls ...
        return controls

    def reset(self):
        # Reset state
        pass
```

Then modify `run_scaling_experiment.py` to use your controller instead of `CentralizedMPC`.

## Problem Formulation

### State Space

- **Payload:** `[x_p, y_p, θ_p]` (position and orientation)
- **Robots:** `[x_i, y_i]` for i = 1..n
- **Global state:** `X ∈ R^(3+2n)`

### Control Space

- **Robot controls:** `[vx_i, vy_i]` for i = 1..n (velocity commands)
- **Global control:** `U ∈ R^(2n)`

### Dynamics (Kinematic)

Payload velocity is the average of robot velocities (uniform pushing assumption):

```
v_p = (1/n) Σ v_i
X_{k+1} = X_k + B(X_k) U_k Δt
```

### Objective Function

```
min Σ (||x_p - x_goal||²_Q + ||u||²_R)
```

Subject to:
- Dynamics constraints
- Velocity limits: `||u_i|| ≤ v_max`

See `.plan/multi_robot_transport_mpc.md` for full mathematical formulation.

## Expected Results

### Scaling Exponent (β)

For centralized MPC with IPOPT:
- **Dense solver:** β ≈ 2.5 - 3.0 (cubic scaling)
- **Sparse solver:** β ≈ 1.5 - 2.0 (better than cubic)

### Practical Limits

Based on MuJoCo's 2.6x real-time factor:
- **n ≤ 16:** Real-time feasible (solve time < 50ms @ 20 Hz)
- **n = 32-64:** Near real-time (100-200ms)
- **n = 128:** Offline only (500ms+)

## Tips for Report

### Figures to Include

1. **Figure 1:** `solve_time_vs_n.pdf` (log-log plot) - main result
2. **Figure 2:** `performance_metrics.pdf` - shows task completion
3. **Figure 3:** `trajectories.pdf` - qualitative behavior

### Key Statistics to Report

From `summary.txt`:
- Scaling exponent β with confidence interval
- Mean solve time for each n
- Success rate vs n
- Crossover point for real-time feasibility

### Discussion Points

1. **Why cubic scaling?** Centralized MPC has dense coupling through payload state
2. **Comparison to distributed:** If you implement distributed controller, compare β values
3. **Practical implications:** Real-time feasible up to n=10-20 for 20Hz control
4. **Future work:** Sparse formulations, ADMM, or distributed MPC for better scaling

## Troubleshooting

### MPC solve failures

If you see "MPC solve failed" messages:
- Reduce horizon: `--horizon 10`
- Increase tolerance: Edit `centralized_mpc.py`, increase `ipopt.tol`
- Increase max iterations: `ipopt.max_iter`

### Slow simulations

- Run without viewer: remove `--viewer` flag
- Reduce max simulation time: `-t 30`
- Use smaller horizon: `--horizon 15`

### Memory issues (large n)

For n > 64:
- Use sparse solver if available
- Reduce horizon to 10-15
- Consider splitting into multiple smaller experiments

## References

- **MPC formulation:** See `.plan/multi_robot_transport_mpc.md`
- **MuJoCo validation:** See `.report/mujoco_evaluation.md`
- **CasADi documentation:** https://web.casadi.org/
- **Problem definition:** Based on discussion in `.plan/` directory

## Citation

If using this code for your research:

```
Harris (2024). "Computational Scaling of Centralized MPC for Multi-Robot
Payload Transport." Y3 Dot Project, Swarm Carry.
```

## Contact

For questions or issues with this experimental package, refer to the main project documentation.
