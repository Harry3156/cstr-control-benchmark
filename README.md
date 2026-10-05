# PID vs. LQR vs. Linear MPC vs. Nonlinear MPC, with EKF State Estimation, on a Benchmark CSTR

## Problem
This project controls the Klatt & Engell (1998) CSTR benchmark — a 4th-order
nonlinear reactor producing cyclopentenol (B) from cyclopentadiene (A), with
a consecutive side reaction (B→C) and a parallel side reaction (2A→D). The
reactor exhibits non-minimum-phase (inverse-response) behavior and is a
well-known benchmark in the nonlinear process control literature.

- **States:** cA, cB (mol/L), reactor temperature θ, coolant temperature θK
- **Measured:** cB and θ only — cA and θK are not measured
- **Manipulated:** F (feed dilution rate, 5–35 /h), QK (cooling rate, −8500–0 kJ/h)
- **Model validated** against the paper's published operating point
  (cA=1.235, cB=0.90, θ=134.14°C, θK=128.95°C) to within numerical tolerance.

Reference: Klatt, K.-U. and Engell, S. (1998). "Gain-scheduling trajectory
control of a continuous stirred tank reactor." *Computers & Chemical
Engineering*, 22(4–5), 491–502.

## Method
- RGA computed two ways (step test, and linearization) — both ≈0.57,
  confirming the pairing cB↔F, θ↔QK, with real but moderate loop interaction.
- All controllers tested on the same setpoint sequence from the paper's own
  Fig. 4: cB steps 0.90 → 0.70 → 0.95 mol/L, θ held at 134.14°C.
- **PID** — two independent PI loops (cB↔F, θ↔QK), anti-windup.
- **LQR** — designed at the main operating point, with true-equilibrium
  feedforward (a reference state/input that is an actual steady state of
  the reactor, required since LQR has no integral action of its own).
- **Linear MPC** (CVXPY) — same linearization as LQR, receding horizon,
  input constraints enforced explicitly, plus a θ ≤ 134.3°C output
  constraint tested as a stress case.
- **Nonlinear MPC** (GEKKO) — full nonlinear model used directly for
  prediction, same constraints and setpoint sequence. Solved as a single
  full-horizon optimal control problem (see limitations below).
- **EKF** — paired with LQR; estimates the unmeasured states (cA, θK) from
  noisy cB/θ measurements only, started from a deliberately wrong guess.
- **Disturbance test** — same sequence rerun with the unmeasured feed
  concentration cA0 at its extremes (4.5, 5.7 vs. nominal 5.1).

## Key Results

![PID](plots/PID.png)
![LQR](plots/LQR.png)
![Linear MPC, theta-constrained](plots/mpc.png)
![Nonlinear MPC](plots/nmpc.png)
![EKF](plots/ekf.png)

| | PID | LQR | Linear MPC | Nonlinear MPC |
|---|---|---|---|---|
| Final cB (target 0.95) | ~0.93 (short) | 0.9500 | 0.9500 | 0.9500 |
| θ upper limit (134.3°C) | not enforced | not enforced | 134.32°C | **134.300°C** |
| Model used for prediction | none | fixed linear | fixed linear | true nonlinear |

**Finding 1:** PID is the clear baseline — real steady-state shortfall and
the widest θ swings, consistent with this reactor's known non-minimum-phase
difficulty.

**Finding 2:** With *only* input saturation active, LQR and linear MPC
perform nearly identically — clipping an optimal gain after the fact is
close to as good as planning around the limit, for input constraints alone.

**Finding 3 (headline result):** With a real *output* constraint
(θ ≤ 134.3°C), linear MPC cuts the constraint violation by ~90% versus LQR
(134.51°C → 134.32°C), because the limit is enforced inside the optimization
over the whole prediction horizon, not clipped after the fact.

**Finding 4:** Nonlinear MPC very nearly eliminates the remaining violation
(134.300°C vs. linear MPC's 134.32°C), because it predicts using the true
nonlinear dynamics instead of one fixed linearization — the small residual
gap in linear MPC is genuine model mismatch, not a tuning issue.

**Finding 5 (disturbance rejection):**

| | cA0=5.1 | cA0=4.5 | cA0=5.7 |
|---|---|---|---|
| LQR final cB (target 0.95) | 0.9500 | 0.8979 | 0.9911 |
| MPC final cB (target 0.95) | 0.9500 | 0.8900 | 0.9999 |

Neither LQR nor MPC rejects this disturbance well — both show comparable
steady-state offset (~0.04–0.06 mol/L), since neither has integral action
or a disturbance estimator to detect that the assumed cA0 is wrong. This is
consistent with the original paper's own motivation: its gain-scheduling
reference controller exists specifically to add disturbance robustness that
plain state feedback lacks.

**Finding 6 (state estimation):** An EKF paired with LQR, using only noisy
cB/θ measurements and started from a deliberately wrong initial guess
(cA off by 0.15, θK off by 0.5), converges to within 0.001–0.014 of the
true hidden states within the first few steps, and achieves closed-loop
tracking (final cB = 0.9499) matching the unrealistic full-state-feedback
case (0.9500) to within noise. This closes the project's main standing
assumption: control is demonstrated using only the sensors the problem
statement actually allows.

## Honest limitations / future work
- **Nonlinear MPC is solved as a single full-horizon optimal control
  problem**, not true receding-horizon feedback control. A receding-horizon
  version (re-solving every step) was attempted but hit genuine infeasibility
  under the θ constraint with a short lookahead — itself an interesting
  finding, suggesting linear MPC's near-success partly reflects its linear
  model underestimating the true θ rise.
- **Disturbance rejection is unresolved**, not just untested: the results
  above show a real, quantified limitation. Fixing it needs a disturbance-
  augmented estimator (treating cA0 itself as an estimated state), the
  standard "offset-free MPC" approach in industrial practice — not
  implemented here.
- **Moving Horizon Estimation** was scoped out in favor of EKF, which
  already closes the full-state-feedback gap. MHE's real advantage over EKF
  (constraint-aware estimation, e.g. enforcing cA ≥ 0) is left for Project 3
  of the broader roadmap, where it's the dedicated focus.

## Tools
Python (NumPy, SciPy, python-control, CVXPY, GEKKO)
