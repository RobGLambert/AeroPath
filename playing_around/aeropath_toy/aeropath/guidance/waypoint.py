"""
waypoint.py

Toy guidance: "turn toward a waypoint".

Guidance DOES NOT apply physics.
It only outputs Control(throttle, a_lat_cmd).

Physics later clamps a_lat_cmd and computes actual acceleration. 
"""

from __future__ import annotations
import numpy as np

from aeropath.dynamics.state import State, Control, VehicleParams


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def safe_unit(v: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < eps:
        return np.array([1.0, 0.0], dtype=float)
    return v / n


def waypoint_guidance(s: State, target: np.ndarray, params: VehicleParams) -> Control:
    """
    Steps:
    1) Compute desired direction to target
    2) Compare desired vs current velocity direction
    3) Convert to signed lateral accel request using a gain

    Note: this is a very simple guidance law, but it allows us to test the overall system.
    We can later replace this with a more sophisticated guidance law 
    (like pure pursuit or optimal control) without changing the rest of the system.

    s: current state (position, velocity, time)
    target: target position [x,y]
    params: vehicle parameters (for access to a_lat_max)
    """
    to_target = target - s.r
    desired_dir = safe_unit(to_target)
    vhat = safe_unit(s.v)

    # Signed 2D direction error (z component of cross product vhat x desired)
    heading_err = float(vhat[0] * desired_dir[1] - vhat[1] * desired_dir[0])

    # Gain (toy). Bigger means more aggressive turning.
    k = 120.0
    a_lat_cmd = clamp(k * heading_err, -params.a_lat_max, params.a_lat_max)

    throttle = 1.0 if float(np.linalg.norm(to_target)) > 500.0 else 0.2
    return Control(throttle=throttle, a_lat_cmd=a_lat_cmd)
