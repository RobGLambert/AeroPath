"""
forces.py

This is the "ForceModel": it computes dv/dt (acceleration).

Model:
1) Thrust along velocity direction
2) Quadratic drag using relative air velocity v_rel = v - wind
3) Lateral acceleration command perpendicular to velocity, limited by a_lat_max

The main function is compute_acceleration(), which takes the current state, control input, 
vehicle parameters, thrust profile, and wind function, and returns the total acceleration 
and telemetry for logging/debugging.
"""

from __future__ import annotations
from typing import Callable, Tuple
import numpy as np

from .state import State, Control, VehicleParams, ThrustProfile


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def safe_unit(v: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < eps:
        return np.array([1.0, 0.0], dtype=float)
    return v / n


def perp(vhat: np.ndarray) -> np.ndarray:
    """Rotate vector +90 degrees -> left normal."""
    return np.array([-vhat[1], vhat[0]], dtype=float)


def thrust_at_time(tp: ThrustProfile, t: float) -> float:
    """Piecewise thrust schedule: boost -> sustain -> 0."""
    if t < tp.t_boost:
        return tp.T_boost
    if t < tp.t_boost + tp.t_sustain:
        return tp.T_sustain
    return 0.0


def compute_acceleration(
    s: State,
    u: Control,
    params: VehicleParams,
    tp: ThrustProfile,
    wind_fn: Callable[[float, float, float], np.ndarray],
) -> Tuple[np.ndarray, dict]:
    """
    Returns:
      a_total (np.array shape(2,))
      telemetry (dict) for logging/debugging
    """
    # Wind at current position/time
    W = wind_fn(float(s.r[0]), float(s.r[1]), s.t)

    # Relative air velocity (what the vehicle "feels")
    v_rel = s.v - W

    # Direction of travel and left-normal
    vhat = safe_unit(s.v)
    nhat = perp(vhat)

    # Thrust aligned with vhat
    T = thrust_at_time(tp, s.t) * clamp(u.throttle, 0.0, 1.0)
    a_thrust = (T / params.m) * vhat

    # Quadratic drag opposing v_rel
    speed_rel = float(np.linalg.norm(v_rel))
    Fd = -0.5 * params.rho * params.CDA * speed_rel * v_rel
    a_drag = Fd / params.m

    # Lateral acceleration (turning), limited
    a_lat = clamp(u.a_lat_cmd, -params.a_lat_max, params.a_lat_max)
    a_lateral = a_lat * nhat

    a_total = a_thrust + a_drag + a_lateral

    telemetry = {
        "wind": W,
        "v_rel": v_rel,
        "T": T,
        "Fd": Fd,
        "a_thrust": a_thrust,
        "a_drag": a_drag,
        "a_lateral": a_lateral,
        "a_total": a_total,
    }
    return a_total, telemetry
