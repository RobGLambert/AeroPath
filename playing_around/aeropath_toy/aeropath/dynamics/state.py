"""
state.py

This file contains ONLY "data shapes" (dataclasses).
No physics. No integration. No guidance.

For a 2D point-mass "fighter/missile-ish" model we store:
- position r = [x, y]
- velocity v = [vx, vy]
- time t

Control inputs are:
- throttle (0..1)
- lateral acceleration command (signed, clamped in physics model)
"""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class VehicleParams:
    """
    Core physical / performance parameters. These are all constant for a given vehicle. 

    CDA is the "equivalent drag area":
        CDA = Cd * A_ref
    where Cd is the drag coefficient and A_ref is the reference area.
    This lets us compute drag force as:
    F_drag = 0.5 * rho * CDA * v_rel^2
    where v_rel is the velocity relative to the air (v - wind).

    a_lat_max is lateral acceleration limit. This enforces realistic turning.
    v_max is a speed cap, which prevents numerical instability and models physical limits.
    """
    m: float                 # mass [kg]
    CDA: float               # equivalent drag area [m^2]
    a_lat_max: float         # max lateral acceleration [m/s^2]
    v_max: float             # max speed cap [m/s]
    rho: float = 1.225       # air density [kg/m^3]


@dataclass(frozen=True)
class ThrustProfile:
    """
    Piecewise thrust schedule:
      - boost phase
      - sustain phase
      - then 0 thrust after that

    This is a simple way to model a rocket motor that has a boost phase and then a sustain phase.
    The thrust at any time t can be computed using the thrust_at_time() function in forces.py.

    T_boost: thrust during boost phase [N]
    t_boost: duration of boost phase [s]
    T_sustain: thrust during sustain phase [N]
    t_sustain: duration of sustain phase [s]
    After t_boost + t_sustain, thrust is 0.
    """
    T_boost: float
    t_boost: float
    T_sustain: float
    t_sustain: float


@dataclass(frozen=True)
class State:
    """
    Simulation state at one instant in time.
    r: position vector [x,y] meters
    v: velocity vector [vx,vy] m/s
    t: time in seconds

    Note: we could also store "mass remaining" if we wanted to model mass depletion due to fuel burn.
    For simplicity, we assume constant mass here.
    """
    t: float
    r: np.ndarray            # shape (2,)
    v: np.ndarray            # shape (2,)


@dataclass(frozen=True)
class Control:
    """
    Control inputs for one time step.
    throttle: scales thrust 0..1
    a_lat_cmd: desired lateral accel (signed), clamped in physics model

    Note: we could also have a "heading command" or "angle of attack command" instead of a_lat_cmd.
    For simplicity, we directly command lateral acceleration, which the physics model then applies perpendicular to velocity
    """
    throttle: float          # 0..1
    a_lat_cmd: float         # signed lateral acceleration request [m/s^2]
