"""
integrators.py

Integrators advance the state forward in time.

They do NOT know about thrust or drag.
They only call a derivative function deriv(state) -> (dr, dv).

The main function is rk4_step(), which implements the 
classic 4th-order Runge-Kutta method for numerical integration. It takes the 
current state, a time step dt, and a derivative function, 
and returns the new state after advancing by dt.
"""

from __future__ import annotations
from typing import Callable, Tuple
import numpy as np

from .state import State


def rk4_step(
    s: State,
    dt: float,
    deriv: Callable[[State], Tuple[np.ndarray, np.ndarray]],
) -> State:
    """
    RK4 for:
      dr/dt = v
      dv/dt = a(...)
    where a(...) is computed by the caller and passed in via the deriv function.
    Returns the new state after advancing by dt.
    """
    k1_r, k1_v = deriv(s)

    s2 = State(s.t + dt/2, s.r + k1_r*dt/2, s.v + k1_v*dt/2)
    k2_r, k2_v = deriv(s2)

    s3 = State(s.t + dt/2, s.r + k2_r*dt/2, s.v + k2_v*dt/2)
    k3_r, k3_v = deriv(s3)

    s4 = State(s.t + dt, s.r + k3_r*dt, s.v + k3_v*dt)
    k4_r, k4_v = deriv(s4)

    r_new = s.r + (dt/6) * (k1_r + 2*k2_r + 2*k3_r + k4_r)
    v_new = s.v + (dt/6) * (k1_v + 2*k2_v + 2*k3_v + k4_v)

    return State(s.t + dt, r_new, v_new)
