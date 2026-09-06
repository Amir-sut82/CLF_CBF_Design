# CLF-CBF Safety-Critical Attitude Control for a Bi-Copter

This repository implements a **Control Lyapunov Function (CLF)** and **Control Barrier Function (CBF)** based controller for safety-critical attitude stabilization of a nonlinear bi-copter system.

The project focuses on designing controllers that simultaneously provide:

- **Stability guarantees** using Control Lyapunov Functions
- **Safety guarantees** using Control Barrier Functions
- **Actuator feasibility** through Quadratic Programming optimization

The implemented approaches compare:

1. CLF-only controller
2. Decoupled CLF-CBF safety filter
3. Unified CLF-CBF Quadratic Program controller

---

# 1. System Modeling

The bi-copter attitude dynamics are represented as a nonlinear affine control system:

$$
\dot{x}=f(x)+g(x)u
$$

where:

- $x$ represents the system state
- $u$ represents the control input
- $f(x)$ is the nonlinear drift dynamics
- $g(x)$ describes control influence

The attitude state is:

$$
x=[\phi,\theta,\psi,p,q,r]^T
$$

where:

- $\phi,\theta,\psi$ are Euler angles
- $p,q,r$ are body angular velocities

---

# 2. Control Lyapunov Function (CLF)

A Control Lyapunov Function provides a mathematical condition for stability.

For a Lyapunov candidate $V(x)$, the CLF condition is:

$$
\dot{V}(x)+cV(x)\leq0
$$

where $c>0$ defines the convergence rate.

Using Lie derivatives:

$$
L_fV(x)+L_gV(x)u\leq-cV(x)
$$

The controller is obtained by solving a quadratic optimization problem:

$$
\min_u ||u||^2
$$

subject to the CLF stability constraint.

---

# 3. Control Barrier Function (CBF)

Control Barrier Functions guarantee forward invariance of a safe set.

The safe set is defined as:

$$
C=\{x\in R^n:h(x)\geq0\}
$$

The CBF condition is:

$$
\dot{h}(x)+\alpha(h(x))\geq0
$$

where $\alpha$ is a class-K function.

In this project, the safety objective is maintaining the pitch angle within a predefined limit.

---

# 4. Higher Order Control Barrier Function (HOCBF)

Because the safety output has relative degree greater than one, a Higher Order CBF formulation is used.

The recursive formulation is:

$$
\psi_0=h(x)
$$

$$
\psi_1=\dot{\psi}_0+\alpha_1(\psi_0)
$$

$$
\psi_2=\dot{\psi}_1+\alpha_2(\psi_1)
$$

The final safety constraint is:

$$
\psi_2(x,u)\geq0
$$

---

# 5. CLF-CBF Quadratic Programming Controller

The unified controller solves:

$$
\min_{u,\delta} u^Tu+\rho\delta^2
$$

subject to:

CLF constraint:

$$
L_fV+L_gVu\leq-cV+\delta
$$

CBF constraint:

$$
\psi_2(x,u)\geq0
$$

The slack variable $\delta$ allows a trade-off between perfect convergence and safety feasibility.

---

# 6. Controller Comparison

## CLF-only Controller

Advantages:

- Fast convergence
- Simple optimization problem

Limitation:

- No explicit safety guarantee


## Decoupled CLF-CBF Safety Filter

A nominal CLF controller generates the desired input and the CBF filter modifies it when safety constraints are violated.


## Unified CLF-CBF-QP

The stability and safety objectives are solved simultaneously, providing a balanced solution between:

- Tracking performance
- Safety preservation
- Control effort

---

# 7. Simulation Results

The simulations evaluate the controller performance under different scenarios:

- Attitude stabilization
- Safety constraint enforcement
- Input limitation effects
- Comparison between CLF and CLF-CBF approaches

Expected outcomes:

- CLF achieves convergence but may violate safety constraints
- CBF prevents unsafe states
- Unified CLF-CBF provides simultaneous safety and stabilization

---

# 8. Repository Structure

```
CLF_CBF_Design/
│
├── MATLAB source files
├── Simulation scripts
├── Controller implementations
├── Results and plots
└── README.md
```

---

# 9. Requirements

- MATLAB
- Optimization Toolbox
- Control System Toolbox

---

# 10. Key Concepts

This project demonstrates:

- Nonlinear control
- Lyapunov stability theory
- Safety-critical control
- Control Lyapunov Functions
- Control Barrier Functions
- Quadratic Programming based controllers
- Autonomous system safety guarantees

---

# References

- Ames, A. D. et al. Control Barrier Function Based Quadratic Programs for Safety Critical Systems.
- Khalil, H. K. Nonlinear Systems.
- Cohen, M. and Belta, C. Adaptive and Learning-Based Control of Safety-Critical Systems.
