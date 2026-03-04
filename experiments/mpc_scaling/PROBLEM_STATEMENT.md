# Problem Statement: Centralised MPC for Multi-Robot Payload Transport

We consider $n$ mobile robots cooperatively transporting a rigid payload to a goal position via pushing. The state consists of the payload pose $x_p = [p_x, p_y, \theta]^T \in \mathbb{R}^3$ and robot positions $x_i = [r_{x,i}, r_{y,i}]^T \in \mathbb{R}^2$ for $i = 1, \ldots, n$, giving a global state $X = [x_p^T, x_1^T, \ldots, x_n^T]^T \in \mathbb{R}^{3+2n}$. Each robot is controlled by velocity commands $u_i = [v_{x,i}, v_{y,i}]^T \in \mathbb{R}^2$, collected into a global control vector $U = [u_1^T, \ldots, u_n^T]^T \in \mathbb{R}^{2n}$.

The dynamics are kinematic. Each robot evolves as $x_{i,k+1} = x_i + u_i \Delta t$, and the payload velocity is the average of robot velocities: $\dot{p} = \frac{1}{n} \sum_{i=1}^n u_i$, with no rotation ($\dot{\theta} = 0$) since robots push uniformly along one face. This gives $X_{k+1} = X_k + B(X_k) U_k \Delta t$ where $B \in \mathbb{R}^{(3+2n) \times 2n}$ has dense rows coupling all robot velocities to the payload state.

At each time step $k$, the centralised MPC solves over a horizon of $T$ steps:

$$\min_{U_{0:T-1}} \sum_{t=0}^{T-1} \left( Q_{\text{pos}} \| p_t - p_{\text{goal}} \|^2 + R \| U_t \|^2 \right) + 10 Q_{\text{pos}} \| p_T - p_{\text{goal}} \|^2$$

subject to the dynamics $X_{t+1} = X_t + B(X_t) U_t \Delta t$ for $t = 0, \ldots, T-1$, initial condition $X_0 = X_{\text{measured}}$, and velocity limits $\| u_{i,t} \| \leq v_{\max}$ for all $i, t$. Here $Q_{\text{pos}} = 10$, $R = 0.1$, $\Delta t = 0.05$ s, $T = 20$, and $v_{\max} = 1.0$ m/s are default parameters.

This nonlinear program has $2nT$ decision variables and $nT$ inequality constraints. The dense coupling in $B$ creates $O(n^2)$ nonzeros in the constraint Jacobian, leading to an expected solve time scaling of $\tau \sim \alpha n^\beta$ with $\beta \in [2, 3]$ for standard interior-point solvers. The research objective is to empirically measure $\beta$ as $n$ varies from 2 to 128.
