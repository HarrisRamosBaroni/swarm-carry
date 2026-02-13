# Problem Statement: Distributed Target Estimation via GBP

We consider $n$ agents estimating a shared target position $x \in \mathbb{R}^2$. Agents communicate only with neighbors defined by a graph $\mathcal{G} = (\mathcal{V}, \mathcal{E})$ where $\mathcal{V} = \{1, \ldots, n\}$ and $(i,j) \in \mathcal{E}$ if agents $i$ and $j$ can exchange messages.

## Observation Model (Simulation)

Each agent $i$ receives a single noisy observation of the true target $x_{\text{true}}$:
$$z_i = x_{\text{true}} + \varepsilon_i, \quad \varepsilon_i \sim \mathcal{N}(0, \sigma^2 I_2)$$

Default parameters: $x_{\text{true}} = [5, 3]^T$, $\sigma = 0.5$.

## Factor Definitions

**Observation factor** (one per agent): Encodes likelihood of observation given target position.
$$f_i(x) = \mathcal{N}(z_i \mid x, \sigma^2 I) \propto \exp\left(-\frac{1}{2\sigma^2}\|x - z_i\|^2\right)$$

In information form: precision $\Lambda_{\text{obs}} = \sigma^{-2} I_2$, information vector $\eta_{\text{obs},i} = \Lambda_{\text{obs}} z_i$.

**Consensus factor** (one per edge): Encodes soft constraint that neighboring agents should agree.
$$g_{ij}(x_i, x_j) \propto \exp\left(-\frac{\lambda}{2}\|x_i - x_j\|^2\right)$$

In the demo, consensus is enforced implicitly through GBP message passing rather than explicit $\lambda$. The cavity message $m_{i \to j}$ sent from agent $i$ to $j$ carries $i$'s belief about $x$ (excluding $j$'s contribution), and upon convergence all $x_i \approx x_j$.

## Belief Representation

Each agent maintains a Gaussian belief $b_i(x) = \mathcal{N}^{-1}(\eta_i, \Lambda_i)$ in information form, where $\eta_i = \Lambda_i \mu_i$ is the information vector and $\Lambda_i$ is the precision matrix.

Gaussian Belief Propagation (GBP) proceeds in synchronous rounds. At each iteration $k$:

1. **Message computation:** Each agent computes the cavity distribution (belief excluding neighbor's contribution):
$$m_{i \to j}^{(k)}(x) = \frac{b_i^{(k)}(x)}{m_{j \to i}^{(k-1)}(x)}$$
In information form: $\eta_{i \to j} = \eta_i - \eta_{j \to i}$, $\Lambda_{i \to j} = \Lambda_i - \Lambda_{j \to i}$.

2. **Message exchange:** Agents send $m_{i \to j}$ to all neighbors $j \in \mathcal{N}(i)$ via the communication backend.

3. **Belief update:** Each agent fuses its observation with incoming messages:
$$b_i^{(k+1)}(x) \propto f_i(x) \prod_{j \in \mathcal{N}(i)} m_{j \to i}^{(k)}(x)$$
In information form: $\eta_i = \eta_{\text{obs},i} + \sum_j \eta_{j \to i}$, $\Lambda_i = \Lambda_{\text{obs}} + \sum_j \Lambda_{j \to i}$.

Convergence is achieved when $\|\mu_i - \mu_j\| < \varepsilon$ for all $(i,j) \in \mathcal{E}$. Upon convergence, all agents' beliefs approximate the centralized solution $\hat{x}_{\text{central}} = \frac{1}{n}\sum_{i=1}^n z_i$ (for uniform observation precision).

The research objective is to validate the communication backend and GBP implementation by measuring: (1) iterations to convergence vs. topology, (2) consensus quality vs. centralized fusion, and (3) message complexity vs. agent count. This provides the foundation for distributed control where agents must reach consensus on shared state (e.g., payload position) before computing coordinated actions.
