import math
from dataclasses import dataclass
from typing import Callable, Tuple

import numpy as np


# -----------------------------
# Data containers (state/params)
# -----------------------------
@dataclass
class VehicleParams:
    m: float  # kg 
    CDA: float  # m^2 (Cd * A_ref lumped)
    a_lat_max: float  # m/s^2 (lateral accel limit, e.g. 25g -> ~245)
    v_max: float  # m/s (sanity cap)
    rho: float = 1.225  # kg/m^3


@dataclass
class ThrustProfile:
    T_boost: float  # N (boost thrust)
    t_boost: float  # s (duration of boost)
    T_sustain: float  # N (sustain thrust)
    t_sustain: float  # s (duration after boost)


@dataclass
class State:
    t: float       # time in seconds
    r: np.ndarray  # position [x, y] in m
    v: np.ndarray  # velocity [vx, vy] in m/s


@dataclass
class Control:
    throttle: float  # 0..1 (scales thrust)
    a_lat_cmd: float  # signed lateral accel request (m/s^2)


# -----------------------------
# Environment model (wind)
# -----------------------------
def wind_shear(x: float, y: float, t: float) -> np.ndarray:
    """Example wind field: a mild shear in x with y."""
    # wind to the +x, increasing with y
    wx = 10.0 + 0.002 * y  # m/s
    wy = 0.0
    return np.array([wx, wy], dtype=float)


# -----------------------------
# Thrust model (piecewise)
# -----------------------------
def thrust_at_time(tp: ThrustProfile, t: float) -> float:
    if t < tp.t_boost:
        return tp.T_boost
    if t < tp.t_boost + tp.t_sustain:
        return tp.T_sustain
    return 0.0


# -----------------------------
# Physics helpers
# -----------------------------
def safe_unit(v: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < eps:
        return np.array([1.0, 0.0], dtype=float)  # arbitrary
    return v / n


def perp(vhat: np.ndarray) -> np.ndarray:
    """Rotate vector +90 degrees."""
    return np.array([-vhat[1], vhat[0]], dtype=float)


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


# -----------------------------
# Force / acceleration model
# -----------------------------
def acceleration(
    s: State,
    u: Control,
    params: VehicleParams,
    tp: ThrustProfile,
    wind_fn: Callable[[float, float, float], np.ndarray],
) -> np.ndarray:
    """
    Returns total acceleration dv/dt given state, control, and environment.
    Model:
      - thrust aligned with current velocity direction
      - quadratic drag based on v_rel = v - wind
      - commanded lateral accel applied perpendicular to velocity, capped by a_lat_max
    """
    # Wind at current location/time
    W = wind_fn(float(s.r[0]), float(s.r[1]), s.t)

    # Relative air velocity
    v_rel = s.v - W

    # Unit vectors
    vhat = safe_unit(s.v)  # direction of motion over ground
    nhat = perp(vhat)  # left-normal direction

    # Thrust (aligned with vhat)
    T = thrust_at_time(tp, s.t) * clamp(u.throttle, 0.0, 1.0)
    a_thrust = (T / params.m) * vhat

    # Quadratic drag (opposes relative airflow)
    speed_rel = float(np.linalg.norm(v_rel))
    # Fd = -0.5 * rho * CDA * |v_rel| * v_rel
    Fd = -0.5 * params.rho * params.CDA * speed_rel * v_rel
    a_drag = Fd / params.m

    # Lateral acceleration command (perpendicular to motion)
    a_lat = clamp(u.a_lat_cmd, -params.a_lat_max, params.a_lat_max)
    a_lateral = a_lat * nhat

    # Total acceleration
    return a_thrust + a_drag + a_lateral


# -----------------------------
# RK4 integrator for [r, v]
# -----------------------------
def rk4_step(
    s: State,
    u: Control,
    params: VehicleParams,
    tp: ThrustProfile,
    wind_fn: Callable[[float, float, float], np.ndarray],
    dt: float,
) -> State:
    """
    Integrate:
      dr/dt = v
      dv/dt = a(state, control, ...)
    """

    def deriv(state: State) -> Tuple[np.ndarray, np.ndarray]:
        dr = state.v
        dv = acceleration(state, u, params, tp, wind_fn)
        return dr, dv

    # k1
    k1_r, k1_v = deriv(s)

    # k2
    s2 = State(s.t + dt / 2, s.r + k1_r * dt / 2, s.v + k1_v * dt / 2)
    k2_r, k2_v = deriv(s2)

    # k3
    s3 = State(s.t + dt / 2, s.r + k2_r * dt / 2, s.v + k2_v * dt / 2)
    k3_r, k3_v = deriv(s3)

    # k4
    s4 = State(s.t + dt, s.r + k3_r * dt, s.v + k3_v * dt)
    k4_r, k4_v = deriv(s4)

    r_new = s.r + (dt / 6) * (k1_r + 2 * k2_r + 2 * k3_r + k4_r)
    v_new = s.v + (dt / 6) * (k1_v + 2 * k2_v + 2 * k3_v + k4_v)

    # Optional sanity cap on speed
    speed = float(np.linalg.norm(v_new))
    if speed > params.v_max:
        v_new = v_new * (params.v_max / speed)

    return State(s.t + dt, r_new, v_new)


# -----------------------------
# Tiny guidance: fly toward waypoint
# -----------------------------
def waypoint_guidance(s: State, target: np.ndarray, params: VehicleParams) -> Control:
    """
    Dumb guidance:
      - throttle 1.0 until near target
      - lateral accel command proportional to cross-track error in heading
    """
    to_target = target - s.r
    desired_dir = safe_unit(to_target)

    vhat = safe_unit(s.v)
    # Signed "heading error" in 2D via cross product z-component:
    # cross(vhat, desired) = vhat_x*des_y - vhat_y*des_x
    heading_err = float(vhat[0] * desired_dir[1] - vhat[1] * desired_dir[0])

    # Proportional lateral accel (tune gain)
    k = 120.0  # (m/s^2) per unit heading_err, arbitrary for demo
    a_lat_cmd = clamp(k * heading_err, -params.a_lat_max, params.a_lat_max)

    # simple throttle logic
    throttle = 1.0 if np.linalg.norm(to_target) > 500.0 else 0.2

    return Control(throttle=throttle, a_lat_cmd=a_lat_cmd)


# -----------------------------
# Demo run
# -----------------------------
if __name__ == "__main__":
    # "Missile-ish" demo parameters (not a real specific missile)
    params = VehicleParams(
        m=150.0,
        CDA=0.020,
        a_lat_max=250.0,  # ~25g
        v_max=900.0,
    )
    tp = ThrustProfile(T_boost=18000.0, t_boost=3.0, T_sustain=4000.0, t_sustain=15.0)

    dt = 0.02
    t_end = 25.0

    s = State(
        t=0.0,
        r=np.array([0.0, 0.0], dtype=float),
        v=np.array([250.0, 0.0], dtype=float),
    )

    target = np.array([8000.0, 3000.0], dtype=float)

    traj = []
    while s.t < t_end:
        u = waypoint_guidance(s, target, params)
        traj.append((s.t, s.r[0], s.r[1], s.v[0], s.v[1]))
        s = rk4_step(s, u, params, tp, wind_shear, dt)

    # Print a tiny summary (toy)
    final_dist = float(np.linalg.norm(target - s.r))
    print(f"Final time: {s.t:.2f}s")
    print(f"Final position: {s.r}")
    print(f"Final speed: {np.linalg.norm(s.v):.1f} m/s")
    print(f"Distance to target: {final_dist:.1f} m")
