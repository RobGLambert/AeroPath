"""
propagator.py

The Propagator is the single "advance one step" function.

It wires:
- ForceModel (compute_acceleration)
- Integrator (rk4_step)

After integration it applies constraints (like speed cap).
"""

from __future__ import annotations
from typing import Callable, Tuple
import numpy as np

from .state import State, Control, VehicleParams, ThrustProfile
from .forces import compute_acceleration
from .integrators import rk4_step


def apply_speed_cap(v: np.ndarray, v_max: float) -> np.ndarray:
    speed = float(np.linalg.norm(v))
    if speed <= v_max:
        return v
    return v * (v_max / speed)


def step(
    s: State,
    u: Control,
    dt: float,
    params: VehicleParams,
    tp: ThrustProfile,
    wind_fn: Callable[[float, float, float], np.ndarray],
) -> Tuple[State, dict]:
    last_telemetry = {}

    def deriv(state: State):
        nonlocal last_telemetry
        a, telem = compute_acceleration(state, u, params, tp, wind_fn)
        last_telemetry = telem
        return state.v, a

    s_new = rk4_step(s, dt, deriv)
    s_new = State(s_new.t, s_new.r, apply_speed_cap(s_new.v, params.v_max))
    return s_new, last_telemetry
