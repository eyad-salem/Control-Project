"""
Target Velocity Profiler based on track curvature.
Calculates maximum safe cornering speeds subject to lateral acceleration limits.
"""

import math  # noqa: F401


class VelocityProfiler:
    """Generates target speed profiles based on track curvature or precomputed data."""

    def __init__(self, default_speed=4.0, max_speed=8.0, max_lat_accel=5.0):
        self.default_speed = default_speed
        self.max_speed = max_speed
        self.max_lat_accel = max_lat_accel

    def compute_target_speed(self, kappa, fallback_speed=None):
        """Calculates curvature-limited velocity: v_max = sqrt(a_lat_max / |kappa|)."""
        # check if there is a fallback speed initialized in the function when called it not then use max_speed
        if fallback_speed is None:
            def_limit = self.max_speed
        else:
            def_limit = fallback_speed
        # check if absolute kappa is 0 then it can use max speed since it's straight road
        if def_limit > self.max_speed:
            def_limit = self.max_speed

        if abs(kappa) < 0.000001:
            v_max = def_limit
        else:
            v_max = math.sqrt(self.max_lat_accel/abs(kappa))
        # checks if speed is over the max then it gets it back to the capped speed , and if it's under zero then it makes it zero
        if v_max > def_limit:
            target_speed = def_limit
        elif v_max < 0.0:
            target_speed = 0.0
        else:
            target_speed = v_max
        return float(target_speed)

        # TODO: Milestone 5.1 — Curvature-Limited Velocity Profiler
        # This controls how fast the car drives based on the road shape.
        # It slows the car down in sharp turns to prevent slipping.
        # Implement the formula to calculate safe speed from curvature, and clamp it.

