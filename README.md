# Wind-Aware Autonomous Flight Simulator

## A Probabilistic, Risk-Constrained 2D Autonomous Navigation Framework

------------------------------------------------------------------------

## Overview

This project implements a physics-based 2D aircraft/drone simulation
framework operating under deterministic and stochastic wind fields. It
integrates:

-   Newtonian flight dynamics
-   Aerodynamic drag modeling
-   Configurable wind field generation (laminar, shear, gust,
    stochastic)
-   Deterministic and probabilistic forecast modeling
-   Closed-loop autopilot control (PID/state-space)
-   Monte Carlo uncertainty propagation
-   Risk-aware decision optimization
-   Obstacle detection and avoidance
-   Fuel/time/risk tradeoff analysis

The objective is to simulate autonomous flight under environmental
uncertainty while balancing mission goals, safety constraints, and
operational costs.

------------------------------------------------------------------------

## Motivation

Autonomous systems operating in atmospheric environments must handle:

-   Wind uncertainty
-   Sensor limitations
-   Dynamic obstacles
-   Resource constraints
-   Risk tradeoffs

This project explores how an autonomous vehicle should reason about
uncertainty, risk, and environmental dynamics while completing mission
objectives.

------------------------------------------------------------------------

## Architecture

Core system modules:

-   sim/ (dynamics, integrators)
-   wind/ (deterministic + stochastic models)
-   control/ (PID + robust controllers)
-   estimation/ (Kalman filtering)
-   mission/ (cost functions + constraints)
-   obstacles/ (geometry + avoidance)
-   experiments/ (scenario runner)

System loop:

1.  Generate wind (truth + forecast)
2.  Propagate vehicle dynamics
3.  Apply controller
4.  Evaluate constraints
5.  Log metrics
6.  Repeat for Monte Carlo samples

------------------------------------------------------------------------

## Project Phases

### Phase 1: Physics Core

-   2D vehicle state representation
-   Drag using relative airspeed
-   Euler + RK4 integration
-   Trajectory visualization

### Phase 2: Wind Modeling

-   Constant, shear, gust models
-   Gaussian stochastic wind
-   Ensemble forecast sampling
-   Wind vector visualizations

### Phase 3: Deterministic Autopilot

-   PID path tracking
-   Stability + convergence testing
-   Error metrics and diagnostics

### Phase 4: Probabilistic Autonomy

-   Distinguish truth vs forecast wind
-   Monte Carlo rollouts
-   Risk-sensitive objective minimization

### Phase 5: Mission Constraints

-   Fuel penalties
-   Time optimization
-   Risk metrics (probability + CVaR)
-   Strategy comparison (Pareto analysis)

### Phase 6: Obstacle Avoidance

-   Static + dynamic obstacles
-   Noisy sensor models
-   Potential fields / Monte Carlo avoidance
-   Collision probability evaluation

------------------------------------------------------------------------

## Mathematical Foundations

-   Newtonian mechanics
-   Numerical ODE integration
-   Linear algebra
-   Control theory
-   Kalman filtering
-   Monte Carlo simulation
-   Risk theory
-   Multi-objective optimization

------------------------------------------------------------------------

## Technology Stack

-   Python 3
-   NumPy
-   SciPy
-   Matplotlib
-   Linux development environment
-   Optional C++ extensions

------------------------------------------------------------------------

## Why This Project Matters

This project demonstrates:

-   Systems-level engineering thinking
-   Applied mathematics
-   Uncertainty quantification
-   Risk-aware autonomous decision-making
-   Clean software architecture
-   Reproducible experimentation

------------------------------------------------------------------------

## Author

Robert G. Lambert\
Halifax, Nova Scotia
