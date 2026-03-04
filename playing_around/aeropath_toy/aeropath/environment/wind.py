"""
wind.py

Wind models are pure functions:

    wind(x, y, t) -> [wx, wy]

Dynamics uses wind through relative air velocity:
    v_rel = v - wind
This file has a simple toy wind model: wind shear that increases with North-South distance. 
"""

import numpy as np


def wind_shear(x: float, y: float, t: float) -> np.ndarray:
    """
    Toy wind shear:
      - wind blows to +x
      - increases with y

    This is a very simple model, but it allows us to test that 
    the dynamics correctly handle spatially varying wind.
    """
    wx = 10.0 * (y / 10.0)  # wind increases by 10 m/s for every 10 m north
    wy = 0.0
    return np.array([wx, wy], dtype=float)
