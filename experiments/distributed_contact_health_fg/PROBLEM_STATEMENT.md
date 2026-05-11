# Problem Statement: Distributed Contact-Health Factor-Graph Controller for Multi-Robot Payload Transport

## Relation to other formulations

This experiment is to
[`centralised_contact_health_fg`](../centralised_contact_health_fg/PROBLEM_STATEMENT.md)
what
[`drcap_fg`](../drcap_fg/PROBLEM_STATEMENT.md)
is to
[`mrcap_fg`](../mrcap_fg/PROBLEM_STATEMENT.md):
the same factor-graph contribution, decentralised onto per-robot local graphs
solved by iterated Gaussian Belief Propagation (GBP) over a neighbour-only
communication backend.

The full physical motivation, contact-sensing model
($F_{base,i}, F_{wall,i}$), force-weighted Procrustes derivation,
contact-health-modulated regulariser $\sigma_u^{\text{eff}}(\bar F_k)$,
per-robot recovery + position-lock terms, ablation design, and hypotheses
(H1–H3) are defined in the centralised problem statement and are
**inherited unchanged**. Only the inference and information layout differ.

## What changes relative to the centralised version

The centralised controller uses three pieces of *globally aggregated*
information:

1. The weighted-Procrustes anchor $\hat{\mathbf{c}}_k$, which needs
   $\{\mathbf{p}_i, w_i, \mathbf{r}_i\}_{i=1}^n$.
2. The mean wall-squeeze residual $\bar F_k = \frac{1}{n}\sum_i F_{wall,i}$,
   which feeds $\sigma_u^{\text{eff}}$ (retained as a contingent safeguard
   — see centralised doc).
3. The per-robot post-solve corrections (bidirectional force-recovery +
   contact-health-gated position lock), which are already local but read
   $\hat{\mathbf{c}}_k$ for the desired-slot computation and $w_i$ (own) for
   the pos-lock gate.

The distributed controller obtains (1) and (2) over the same neighbour-only
communication channel that DR.CAP already runs, by piggybacking compact
**sufficient statistics** on the per-iteration GBP message:

- For weighted Procrustes: each robot broadcasts
  $(\mathbf{p}_i,\, \mathbf{r}_i,\, w_i)$ — five scalars per robot — in
  addition to the DR.CAP belief payload. Each robot computes the global
  weighted means $\bar{\mathbf{p}}, \bar{\mathbf{r}}$ and the
  $2\times2$ cross-covariance $M = \sum_i w_i (\mathbf{p}_i - \bar{\mathbf{p}})(\mathbf{r}_i - \bar{\mathbf{r}})^\top$
  *locally* by accumulating its own contribution and summing over received
  messages from neighbours; the closed-form weighted SVD is then a $2\times2$
  operation done in every robot. With a fully-connected topology (the lab
  default for $n \le 4$), every robot recovers the *exact* centralised
  $\hat{\mathbf{c}}_k$. With a partial topology, the result is a
  neighbourhood-restricted Procrustes; one consensus round on the four
  scalar partial sums recovers the global value if needed.
- For $\bar F_k$: each robot broadcasts $F_{wall,i}$. With full topology
  every robot computes the exact mean; with partial topology the local mean
  is biased toward its neighbourhood (acceptable — $\sigma_u^{\text{eff}}$
  is a soft regulariser, and the centroid-control consensus already present
  in DR.CAP smooths $\mathbf{u}$ across the network).

Because the contact-health pieces are re-derived locally from broadcast
sufficient statistics rather than added as new factors, **the local FG
variable layout, factor types, and message format from DR.CAP are
preserved**. The only structural addition is a small augmented payload on
each outgoing message:

```
GaussianMessage payload (DR.CAP): [own_traj, own_centroid] in canonical form
GaussianMessage payload (this):   [own_traj, own_centroid, p_i, r_i, w_i, F_wall,i]
```

That payload is consumed *outside* the GBP linear system: it sets the
robot's local start-anchor target ($\hat{\mathbf{c}}_k$) and its local
$\sigma_u^{\text{eff}}$ at warm-start time, then GBP runs as in DR.CAP.

## Local factor graph

Identical to DR.CAP, with two FG-level replacements that mirror the
centralised contact-health changes:

| Factor (DR.CAP) | Replacement here |
|---|---|
| Start anchor on $\mathbf{x}_0^c$ at unweighted centroid estimate | Anchor at the *force-weighted Procrustes* estimate $\hat{\mathbf{c}}_k$ computed from broadcast $(\mathbf{p}_j, \mathbf{r}_j, w_j)$ |
| Control regulariser $\sigma_u = 0.3$ | Contact-health-modulated $\sigma_u^{\text{eff}}(\bar F_k) = \sigma_u^0 / (1 + \alpha \, h_k^+)$ from broadcast $\{F_{wall,j}\}$ (contingent — see centralised doc) |

All other factors (reference prior, motion model, terminal anchor, robot
motion, pull-in, R2R distance, centroid consensus) are unchanged from
DR.CAP. No new variables. Per-robot variable count $9N + 6$ unchanged.

The two remaining contact-health channels — bidirectional per-robot
force-recovery and contact-health-gated position-lock — live outside the
FG as post-solve corrections, as in the centralised formulation.

## Per-robot post-solve corrections

The bidirectional force-recovery and contact-health-gated position-lock
terms are already per-robot in the centralised formulation and translate
without modification:

$$
\mathbf{v}_i^{\text{cmd}}
= \mathbf{v}_i^{\text{rigid}}(\mathbf{u}_i^*)
+ \beta\,\bigl(F_{wall}^* - F_{wall,i}\bigr)\, \hat{n}_i
+ (1 - w_i)\,K_p\,\bigl(\hat{\mathbf{p}}_k + R(\hat{\theta}_k)\,\mathbf{r}_i - \mathbf{p}_i\bigr)
$$

where $\mathbf{u}_i^*$ is robot $i$'s local centroid-control estimate (read
from its own GBP-converged graph, exactly as DR.CAP reads its own
$\mathbf{u}_0^{c,*}$), $\hat{\mathbf{c}}_k = (\hat{\mathbf{p}}_k, \hat{\theta}_k)$
is robot $i$'s local weighted-Procrustes estimate, $\hat{n}_i$ is its own
forward axis, and $w_i$ is its own contact-health weight (computed locally
from its own $F_{wall,i}, F_{base,i}$). All inputs are local — no extra
communication beyond what the stats-round message already carries. The
force-recovery sign is automatic (push in when under-pressing, back off
when over-pressing) and the pos-lock gating $(1 - w_i)$ collapses to zero
for healthy robots so that force is the trustworthy signal when contact
exists, with geometry as fallback only when contact is degraded — see the
centralised problem statement for the full rationale.

## Reduction property

When the topology is fully connected (or after one consensus round on the
sufficient statistics), the local weighted-Procrustes estimate at every
robot equals the centralised one and every local $\bar F_k$ equals the
centralised mean. In that regime the controller is **functionally
equivalent** to the centralised contact-health controller, with GBP playing
the role that LM plays in the centralised solve. As in DR.CAP, the
distributed cost is in extra communication (one augmented broadcast per
GBP iteration, neighbour-only) and a small number of GBP iterations to
convergence.

## Scalability

Per-robot local FG: $9N + 6$ variables, identical to DR.CAP. Per-step
overhead beyond DR.CAP is $O(|\text{neighbours}|)$ scalar accumulation for
the weighted-Procrustes statistics and $\bar F_k$ — negligible. Message
size grows by 6 scalars per robot, independent of $n$ or $N$.

## Experiment

Same configuration, ablation, and metrics as the centralised problem
statement: $n \in \{3, 4\}$, face-contact formation, translation goals
(forward $5\,\text{m}$ and diagonal $(3, 2, 0)\,\text{m}$), four controller
conditions, no orientation goals. The hypotheses (H1 grip maintenance, H2
force-weighted centroid estimation, H3 graceful degradation under lateral
`xfrc` disturbance) are identical and tested on per-robot quantities
aggregated post-hoc. As in the centralised doc, **wheel-slip injection is
explicitly not used** as a stressor (forklift contact, not wheel friction,
holds the formation together).

Additional metrics specific to this formulation:
- **GBP iterations** per control step: mean, max (as in DR.CAP).
- **Cross-robot disagreement** on $\hat{\mathbf{c}}_k$ at warm-start
  (max-pairwise over robots) — a sanity check that the broadcast
  sufficient statistics actually produce equivalent local estimates under
  the test topology.
- **Communication overhead**: extra bytes per control step from the
  one-shot stats round, relative to DR.CAP's per-iteration belief payload.
