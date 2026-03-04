"""
plot_2d.py

Visualization only.

Plots:
1) Trajectory (x vs y) + wind vectors sampled along the path
2) Speed vs time
3) Thrust vs time

Assumes the log dict has:
t, x, y, speed, T, windx, windy
"""

from __future__ import annotations
from typing import Iterable, Dict, Any, Optional, Tuple
import numpy as np
import matplotlib.pyplot as plt


def _arr(log: Iterable[Dict[str, Any]], key: str) -> np.ndarray:
    """Extract a numeric column from log into a numpy array."""
    return np.array([row[key] for row in log], dtype=float)


def plot_run_2d(
    log: Iterable[Dict[str, Any]],
    target: Optional[Tuple[float, float]] = None,
    title: str = "AeroPath Toy Run (2D)",
    wind_stride: int = 50,
    wind_scale: Optional[float] = None,
) -> None:
    """
    Parameters
    ----------
    wind_stride:
        Plot one wind arrow every N samples (keeps plot readable).
        If your dt is small, try 50-200.

    wind_scale:
        If arrows are too large/small, set a number like 50, 100, 200.
        If None, matplotlib chooses.
    """
    log = list(log)
    if len(log) == 0:
        raise ValueError("Log is empty — nothing to plot.")

    t = _arr(log, "t")
    x = _arr(log, "x")
    y = _arr(log, "y")
    speed = _arr(log, "speed")
    thrust = _arr(log, "T")
    windx = _arr(log, "windx")
    windy = _arr(log, "windy")

    # ---- 1) Trajectory with wind arrows ----
    plt.figure()
    plt.plot(x, y)
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.title(title)
    plt.axis("equal")
    plt.grid(True)

    if target is not None:
        tx, ty = target
        plt.scatter([tx], [ty], marker="x")
        plt.annotate("Target", (tx, ty))

    # Sample indices for wind arrows
    idx = np.arange(0, len(x), max(1, int(wind_stride)))

    # quiver draws arrows at (x[idx], y[idx]) with components (windx[idx], windy[idx])
    plt.quiver(
        x[idx], y[idx],
        windx[idx], windy[idx],
        angles="xy",
        scale_units="xy",
        scale=wind_scale,
        width=0.003,
    )

    # ---- 2) Speed vs time ----
    plt.figure()
    plt.plot(t, speed)
    plt.xlabel("time (s)")
    plt.ylabel("speed (m/s)")
    plt.title("Speed vs Time")
    plt.grid(True)

    # ---- 3) Thrust vs time ----
    plt.figure()
    plt.plot(t, thrust)
    plt.xlabel("time (s)")
    plt.ylabel("thrust (N)")
    plt.title("Thrust vs Time")
    plt.grid(True)

    plt.show()
