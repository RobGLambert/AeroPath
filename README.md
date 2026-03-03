# AeroPath  
## Wind-Aware Autonomous Flight Simulator  
### A Probabilistic, Risk-Constrained 2D Autonomous Navigation Framework

---

## Overview

AeroPath is a physics-based 2D aircraft/drone simulation framework operating under deterministic and stochastic wind fields. It integrates:

- Newtonian flight dynamics  
- Aerodynamic drag modeling  
- Configurable wind field generation (laminar, shear, gust, stochastic)  
- Deterministic and probabilistic forecast modeling  
- Closed-loop autopilot control (PID / state-space)  
- Monte Carlo uncertainty propagation  
- Risk-aware decision optimization  
- Obstacle detection and avoidance  
- Fuel / time / risk tradeoff analysis  

The primary objective is to simulate autonomous flight under environmental uncertainty while balancing mission goals, safety constraints, and operational costs.

This project integrates concepts from:

- Aerospace engineering  
- Robotics and autonomy  
- Control systems  
- Numerical simulation  
- Uncertainty quantification  
- Risk management  
- Statistical modeling  

---

## Motivation

Autonomous systems operating in atmospheric environments must handle:

- Wind uncertainty  
- Sensor limitations  
- Dynamic obstacles  
- Resource constraints  
- Risk tradeoffs  

Traditional deterministic control methods often fail under forecast uncertainty or environmental disturbances.

This project explores:

> How should an autonomous vehicle reason about uncertainty, risk, and environmental dynamics while completing mission objectives?

It also serves as a demonstration of applied mathematical modeling, systems engineering, and robust decision-making.

---

## High-Level Architecture
```bash
/sim
    dynamics.py
    integrators.py
/wind
    deterministic.py
    stochastic.py
    ensemble.py
/control
    pid.py
    robust_controller.py
/estimation
    kalman.py
/mission
    cost_functions.py
    constraints.py
/obstacles
    geometry.py
    avoidance.py
/experiments
    scenario_runner.py
```


### Core System Loop

1. Generate wind field (truth + forecast)  
2. Propagate vehicle dynamics  
3. Apply controller  
4. Evaluate constraints  
5. Log metrics  
6. Repeat for Monte Carlo samples  

---

# Step 1: Physics-Based Flight Model

## State Representation

2D vehicle state vector:
$` x = [x, y, vx, vy] `$


Optional altitude extension:
$` x = [x, y, z, vx, vy, vz] `$


## Forces Modeled

- Thrust  
- Quadratic aerodynamic drag  
- Wind advection  
- Gravity (if 3D)  

Drag uses relative airspeed:
```math
\boldsymbol{v}_{air} = \boldsymbol{v} - \boldsymbol{w}
```
```math
\boldsymbol{F}_{drag} = -k \dot \lVert \boldsymbol{v_{air}} \rVert \dot v_{air}
```

## Numerical Integration

- Forward Euler (baseline)  
- RK4 (improved accuracy)  
- Step-size stability analysis  

### Deliverables

- Trajectory plots (x–y)  
- State evolution over time  
- Energy / speed diagnostics  

---

# Step 2: Wind Field Modeling

## Deterministic Models

- Constant wind  
- Shear wind  
- Structured gust fronts  
- Sinusoidal disturbances  

## Stochastic Models

- Gaussian perturbations  
- Time-correlated noise  
- Spatially varying random fields  

## Ensemble Forecast Modeling

Wind forecast represented as:
```math
{ w(1), w(2), ..., w(N) }
```


Monte Carlo sampling propagates trajectory uncertainty.

### Deliverables

- Vector field visualizations  
- Ensemble wind visualizations  
- Wind statistics  

---

# Step 3: Deterministic Autopilot

## Control Strategy

- PID-based position tracking  
- Velocity damping  
- Path-following controller  

Control objective:
```math
min || x - x_goal ||
```

### Performance Metrics

- Tracking error  
- Convergence time  
- Overshoot  
- Stability under disturbance  

---

# Step 4: Probabilistic Autonomy

Distinction between:

- **Truth wind** (environment)  
- **Forecast wind** (belief model)  

## Robust Decision Strategies

- Mean forecast control  
- Worst-case control  
- Expected value minimization  
- Risk-sensitive objective  

Monte Carlo rollout used for decision selection.

Objective:
```math
min E[J] + λ Var(J)
```

Where:

- `J` = mission cost  
- `λ` = risk aversion parameter  

---

# Step 5: Real-World Constraints

## Fuel / Energy

- Penalize thrust magnitude  
- Energy budget constraints  

## Time

- Penalize mission duration  

## Risk

- Probability of mission failure  
- Probability of leaving safety corridor  
- Conditional Value at Risk (CVaR)  

## Multi-Objective Optimization

Pareto front comparison between:

- Fastest  
- Safest  
- Most fuel efficient  

---

# Step 6: Obstacle Detection and Avoidance

## Obstacle Representation

- Circular obstacles  
- Moving obstacles  
- Random debris fields  

## Sensing Models

- Perfect information  
- Noisy sensor measurements  
- Limited detection range  
- Update frequency constraints  

## Avoidance Strategies

- Potential fields  
- Velocity obstacles  
- Monte Carlo forward simulation  
- Risk-aware trajectory selection  

## Safety Metrics

- Collision probability  
- Minimum separation distance  
- Mission success rate  

---

# Mathematical Foundations

- Newtonian mechanics  
- Numerical ODE integration  
- Linear algebra  
- Control theory  
- Kalman filtering  
- Monte Carlo simulation  
- Risk theory  
- Optimization  

---

# Software Design Principles

- Modular architecture  
- Reproducible experiments  
- Deterministic seeds  
- Unit tests for physics modules  
- Benchmarking of integrators  
- Clear documentation  
- Explicit assumptions documented  

---

# Example Experiments

- Straight-line mission under shear wind  
- Mission under stochastic gust field  
- Robust vs deterministic controller comparison  
- Obstacle crossing with limited sensor range  
- Fuel-constrained route planning  

---

# Evaluation Metrics

- Tracking error distribution  
- Collision rate  
- Fuel consumption  
- Time to objective  
- Risk measures  
- Computational performance  

---

# Technology Stack

- Python 3.x  
- NumPy  
- SciPy  
- Matplotlib  
- Optional: C++ extensions for performance  
- Linux-based development environment  

---

# Future Extensions

- 3D flight dynamics  
- Nonlinear aerodynamic modeling  
- Model Predictive Control  
- Reinforcement learning agent  
- Real weather dataset ingestion  
- Real-time visualization  
- Embedded implementation  

---

# Why This Project Matters

This project demonstrates:

- Applied mathematical modeling  
- Engineering systems thinking  
- Uncertainty quantification  
- Risk-aware autonomy  
- Clean software architecture  
- Technical documentation ability  

This is not a toy simulator.

It is a structured exploration of autonomous decision-making under environmental uncertainty.

---

# Author

**Robert G. Lambert**  
Applied Mathematics & Physics  
Meteorology  
Halifax, Nova Scotia  
