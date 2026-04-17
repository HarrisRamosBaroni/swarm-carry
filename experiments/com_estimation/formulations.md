# Centre-of-Mass Estimation via Vertical Force Readings

---

## Symbol glossary

Four categories — be precise about which is which.

### Sensor measurements (read from hardware each timestep)

| Symbol | Description |
|--------|-------------|
| $F^z_i$ | Tared vertical force at robot $i$'s fork-base sensor (N). One scalar per robot per timestep. |

### Known constants / precomputed parameters (not estimated, not in the FG)

| Symbol | Description |
|--------|-------------|
| $\mathbf{p}_i \in \mathbb{R}^2$ | World-frame position of robot $i$'s fork base, computed from the fixed formation offsets plus current payload odometry. Updated each timestep but treated as a known input. |
| $\sigma_\text{force},\, \sigma_\text{prior},\, \sigma_\text{drift}$ | Noise hyperparameters. Fixed scalars set at construction time. |

### Factor graph variables (live in the FG, what the solver optimises over)

| Symbol | Description |
|--------|-------------|
| $\mathbf{c}_k \in \mathbb{R}^2$ | CoM position $[x_\text{com},\, y_\text{com}]$ in the **payload frame** at timestep $k$. This is the only variable in Formulation A. |
| $\mathbf{c}_j \in \mathbb{R}^2$ | Same quantity but at planning horizon step $j$ in Formulation B. One variable node per horizon step. |

### Symbols from the planning FG — distinct from the above

| Symbol | Description |
|--------|-------------|
| $\mathbf{C}_j \in \mathbb{R}^3$ | Centroid **pose** $[x, y, \theta]$ of the robot formation — the planning variable from MR.CAP/DR.CAP. Completely separate from $\mathbf{c}_j$. $\mathbf{C}_j$ is the geometric centre of the robots; $\mathbf{c}_j$ is the CoM of the payload. |

### Derived / output

| Symbol | Description |
|--------|-------------|
| $\hat{\mathbf{c}}_k$ | The FG's posterior mean estimate of $\mathbf{c}_k$ after solving. This is what gets read out and passed to the planner. |

---

## Background: closed-form baseline

Static moment balance gives a direct solution:

$$
\mathbf{c} = \frac{\sum_i F^z_i \, \mathbf{p}_i}{\sum_i F^z_i}
$$

This is exact under noiseless, static conditions. A factor graph replaces this
with a principled least-squares formulation that (a) handles measurement noise,
and (b) tracks a time-varying CoM. The factor formulation below also avoids
computing any global sum, making it directly suitable for decentralised use.

---

## Formulation A — Separate estimator (recommended for our scenario)

Run a dedicated estimation factor graph at each control timestep $k$.
Output $\hat{\mathbf{c}}_k$ is passed to the planning controller as a fixed parameter.

### Variables

| Symbol | Dim | Meaning |
|--------|-----|---------|
| $\mathbf{c}_k \in \mathbb{R}^2$ | 2 | CoM position $[x,y]$ in payload frame at timestep $k$ |

### Factors

> In all factors below, $\mathbf{c}_k$ is the **FG variable** (what the solver touches).
> $F^z_i$ and $\mathbf{p}_i$ are **constants** baked into each factor at graph-build time.
> No global sum is computed — each robot only needs its own $F^z_i$.

**1. Measurement factor** — one per robot $i$, connected to $\mathbf{c}_k$ only

Each robot contributes a precision-weighted prior: "I think the CoM is at my
fork position $\mathbf{p}_i$, and I am confident in proportion to the load I am bearing."

$$
\mathbf{e}^{\text{meas}}_i(\mathbf{c}_k) = \mathbf{c}_k - \mathbf{p}_i, \qquad
\Lambda_i = \frac{F^z_i}{\sigma_\text{force}^2}\, I_2
$$

The FG combines these by summing precisions. The posterior mean is:

$$
\hat{\mathbf{c}}_k = \left(\sum_i \Lambda_i\right)^{-1} \sum_i \Lambda_i\, \mathbf{p}_i
= \frac{\sum_i F^z_i\, \mathbf{p}_i}{\sum_i F^z_i}
$$

which is exactly the closed-form result — $\sigma_\text{force}^2$ cancels out.
No robot ever needs to know the total weight $W$; the normalisation emerges
from the precision-sum at the variable node. This is also the natural form
for GBP message passing in a decentralised system.

**2. Prior factor** — regularisation toward geometric centre, connected to $\mathbf{c}_k$ only

$$
\mathbf{e}^{\text{prior}}(\mathbf{c}_k) = \mathbf{c}_k - \mathbf{0}, \qquad
\Sigma_\text{prior} = \sigma_\text{prior}^2\, I_2, \quad \sigma_\text{prior} \approx 0.15\,\text{m}
$$

Prevents drift when forces are near-equal and the precision-weighted votes
nearly cancel.

**3. Motion model factor** — dynamic tracking (optional but recommended), connects $\mathbf{c}_{k-1}$ and $\mathbf{c}_k$

$$
\mathbf{e}^{\text{mm}}(\mathbf{c}_k, \mathbf{c}_{k-1}) = \mathbf{c}_k - \mathbf{c}_{k-1}, \qquad
\Sigma_\text{mm} = \sigma_\text{drift}^2\, I_2, \quad \sigma_\text{drift} \approx 0.01\,\text{m/step}
$$

Random-walk model: allows slow CoM drift, penalises sudden jumps.
Omit if treating each timestep as independent — in that case $\mathbf{c}_k$ is
re-estimated from scratch each step.

### How the output feeds the planner

- **Stability check** — flag if $\|\hat{\mathbf{c}}_k\| > d_\text{max}$ (CoM too far off-centre).
- **Load rebalancing** — scale per-robot velocity commands by $1/F^z_i$ to redistribute load toward uniform bearing.

### Complexity

Linear FG (measurement model is linear in $\mathbf{c}$). Solves in one LM step, or
analytically via the normal equations. Very low overhead per timestep.
In a decentralised system each robot sends one GBP message to the shared
$\mathbf{c}_k$ variable node; no all-to-all communication is needed.

---

## Formulation B — CoM integrated into the planning factor graph

Extend the MR.CAP / DR.CAP planning FG to include $\mathbf{c}_j$ as a variable at
each horizon step $j = k,\ldots,k+N$.

### Additional variables

| Symbol | Dim | Meaning |
|--------|-----|---------|
| $\mathbf{c}_j \in \mathbb{R}^2$ | 2 | CoM position $[x,y]$ in payload frame at horizon step $j$ |

### Additional factors

**1. CoM measurement factor** — at current step $j = k$ only, same form as Formulation A

$$
\mathbf{e}^{\text{meas}}_i(\mathbf{c}_k) = \mathbf{c}_k - \mathbf{p}_i, \qquad
\Lambda_i = \frac{F^z_i}{\sigma_\text{force}^2}\, I_2
$$

**2. CoM motion model** — across horizon, connects adjacent $\mathbf{c}_j$ nodes

$$
\mathbf{e}^{\text{mm}}(\mathbf{c}_{j+1}, \mathbf{c}_j) = \mathbf{c}_{j+1} - \mathbf{c}_j, \qquad
\Sigma_\text{mm} = \sigma_\text{drift}^2\, I_2
$$

**Note on coupling to $\mathbf{C}_j$** — $\mathbf{c}_j$ does **not** appear as a factor on the
planning centroid $\mathbf{C}_j$. The geometric centroid is steered to the goal
independently of where the mass is. The CoM estimate instead feeds a
post-solve load-balancing step that scales per-robot velocity commands.

### When this is warranted

Only if the planner needs to *anticipate* future CoM shifts — e.g., contents
that move during a turn, or a liquid payload with known sloshing dynamics.
For a rigid box $\mathbf{c}_j$ will be nearly constant across the horizon and
Formulation A achieves the same result at lower complexity.

---

## Comparison

| | Formulation A | Formulation B |
|--|--|--|
| FG structure | separate, 2D variable | integrated into planning FG |
| Planning FG changes | none | $+N$ variables, $+2$ factor types |
| Handles static CoM | yes | yes |
| Handles drifting CoM | yes (motion model) | yes |
| Handles anticipated future CoM shift | no | yes |
| Requires global force sum $W$ | no | no |
| Decentralised-ready (GBP) | yes | yes |
| Implementation complexity | low | medium |
| Recommended for rigid box payload | **yes** | no |

---

## Practical caveat — dynamic phases

The moment-balance model assumes the payload is in **static equilibrium**.
During acceleration, inertial forces corrupt the estimate:

$$
\sum_i F^z_i\, \mathbf{p}_i = mg\,\mathbf{c} + m\,\mathbf{a}_\text{payload} \times (\text{inertial correction})
$$

Mitigations:
- Trust the estimate only after the settle phase ($t > t_\text{settle}$).
- Correct for measured payload acceleration if an IMU is available.
- Use a large $\sigma_\text{drift}$ during drive phases so the estimate reverts toward the prior rather than tracking inertial artefacts.
