"""
demo_intercept.py

This is a simple "executive" that runs a complete simulation of an intercept scenario using the AeroPath toy model.

This is the "Executive":
- defines params, initial state, time step, end time
- loops:
    guidance -> propagator -> log
- prints a short summary
- plots trajectory + wind vectors + time series

This is a "toy" demo, so the guidance is very simple (turn toward the target) 
and the wind is a simple shear.
The purpose is to test that all the pieces work together and to visualize the 
behavior in a simple scenario. 
We can later replace the guidance and wind with more sophisticated models 
without changing the overall structure of this executive.
"""

import numpy as np

from aeropath.dynamics.state import State, VehicleParams, ThrustProfile
from aeropath.dynamics.propagator import step
from aeropath.environment.wind import wind_shear
from aeropath.guidance.waypoint import waypoint_guidance
from aeropath.viz.plot_2d import plot_run_2d


def main():
    # --- Vehicle parameters (toy, not a specific real platform) ---
    # These are constant parameters that define the physical characteristics and performance limits of the vehicle.
    params = VehicleParams(
        m=150.0,
        CDA=0.020,
        a_lat_max=250.0,   # ~25g
        v_max=900.0,
    )

    # --- Thrust profile: boost then sustain then burnout --- 
    # This is a simple way to model a rocket motor that has a boost phase and then a sustain phase.
    tp = ThrustProfile(
        T_boost=18000.0, t_boost=3.0, # 18000 N for 3 seconds
        T_sustain=4000.0, t_sustain=15.0 # then 4000 N for 15 seconds
    )

    # --- Simulation settings ---
    # Time step and end time for the simulation loop.
    dt = 0.02
    t_end = 25.0

    # --- Initial state ---
    # Starting at the origin, flying east at 250 m/s. We want to intercept a target at (8000, 3000).
    # The initial state includes position, velocity, and time. 
    # We could also include "mass remaining" if we wanted to model fuel burn, 
    # but for simplicity we assume constant mass here.
    # Note: the initial velocity is not directly toward the target, so the guidance will have to turn the vehicle.
    s = State(
        t=0.0,
        r=np.array([0.0, 0.0], dtype=float),
        v=np.array([250.0, 0.0], dtype=float),
    )

    # --- Target waypoint ---
    # The guidance will try to steer the vehicle toward this target position.
    # Note: we could also have a moving target by making this a function of time, but for simplicity it's a fixed point here.
    target = np.array([8000.0, 3000.0], dtype=float)

    # Collect log rows for plotting/debugging
    # Each row is a dict with keys: t, x, y, vx, vy, speed, windx, windy, T
    # We log just enough to visualize the trajectory, wind vectors, and time series of speed and thrust.
    # Note: we could log more telemetry (like heading error, acceleration, etc.) if we wanted to analyze the behavior in more detail.
    # For simplicity, we log only a few key variables that are relevant for understanding the intercept performance and the effect of wind.
    log = []

    while s.t < t_end:
        # 1) Guidance chooses control command based on current state and target
        u = waypoint_guidance(s, target, params)

        # 2) Physics propagates state forward one dt given control command and environment (wind)
        s, telem = step(s, u, dt, params, tp, wind_shear)

        # 3) Log just enough to visualize + debug 
        log.append({
            "t": s.t,
            "x": float(s.r[0]),
            "y": float(s.r[1]),
            "vx": float(s.v[0]),
            "vy": float(s.v[1]),
            "speed": float(np.linalg.norm(s.v)),
            "windx": float(telem["wind"][0]),
            "windy": float(telem["wind"][1]),
            "T": float(telem["T"]),
        })

    # Summary statistics at the end of the run
    dist = float(np.linalg.norm(target - s.r))
    print(f"Final t={s.t:.2f}s")
    print(f"Final position: x={s.r[0]:.1f} m, y={s.r[1]:.1f} m")
    print(f"Final speed: {np.linalg.norm(s.v):.1f} m/s")
    print(f"Distance to target: {dist:.1f} m")

    # Plot everything (trajectory + wind vectors + time series) in one figure for easy analysis.
    plot_run_2d(
        log,
        target=(float(target[0]), float(target[1])),
        title="Toy intercept with shear wind",
        wind_stride=80,     # adjust if too many arrows
        wind_scale=None     # set e.g. 200 if arrows too long
    )


if __name__ == "__main__":
    # Run the main function to execute the simulation and visualization.
    main()
