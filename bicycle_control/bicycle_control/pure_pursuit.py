"""
High-Level Lateral Steering Controller: Geometric Pure Pursuit.
Calculates steering curvature from lookahead arc geometry.
"""

import math  # noqa: F401
import numpy as np  # noqa: F401


class PurePursuitController:
    """Adaptive Pure Pursuit lateral controller."""

    def __init__(self, wheelbase=1.25, kv=0.25, l_min=0.8, l_max=2.5,
                 max_steer_rad=math.radians(35.0)):
        self.L = wheelbase
        self.kv = kv
        self.l_min = l_min
        self.l_max = l_max
        self.max_steer_rad = max_steer_rad

    def compute_lookahead(self, v):
        """Adaptive lookahead distance: Ld = clip(kv * v + l_min, l_min, l_max)."""
        ld = self.kv * v + self.l_min
        ld = float(np.clip (ld , self.l_min , self.l_max))
        return ld



        # TODO: Milestone 5.3 Step 1 — Adaptive Lookahead Horizon
        # The car looks further ahead at higher speeds to plan smoother turns.
        # Implement the speed-scaled lookahead formula and clamp it to the allowed range.
        

    def find_target_waypoint(self, x, y, path_points, lookahead):
        """Searches along path for the target waypoint at lookahead distance."""
        if not path_points:
            return 0, (x, y, 0.0)

        # we start by getting the closest point to the car
        min_dist = float('inf')
        nearest_idx = 0
        for i in range(len(path_points)):
            pt = path_points[i]
            dist = math.sqrt((pt[0] - x) ** 2 + (pt[1] - y) ** 2)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i

        # 2. too make sure it keeps goinging until a point is like the same distance of the lookahead 
        for i in range(len(path_points)):
            idx = (nearest_idx + i) % len(path_points)
            pt = path_points[idx]
            dist = math.sqrt((pt[0] - x) ** 2 + (pt[1] - y) ** 2)
            if dist >= lookahead:
                return idx, pt

        # if the point is too far then just gets to the previous one
        fback_idx = (nearest_idx - 1) % len(path_points)
        return fback_idx, path_points[fback_idx]

    def compute_steering(self, x, y, yaw, target_pt, lookahead):
        """Computes steering angle in radians using Pure Pursuit geometry."""
        dx = target_pt[0]-x
        dy = target_pt[1]-y

        d = math.sqrt((dx)**2 + (dy)**2)
        d = max(d , 1e-4) # prevents division by zero by giving a very small number
        y_local = -dx * math.sin(yaw) + dy*math.cos(yaw)
        sin_alpha = y_local / d 

        # steering equation now
        lookahead = max(lookahead,1e-4)#to also prevent division by zero
        s_angle = math.atan(2.0 * self.L * sin_alpha / max(lookahead,1e-3))
        s_angle = float(np.clip(s_angle, -self.max_steer_rad , self.max_steer_rad))

    
        return s_angle



        # TODO: Milestone 5.3 Steps 3 & 4 — Coordinate Transformation & Arc Law
        # This is the core of Pure Pursuit: transform the target into the vehicle's
        # local frame, then use the arc geometry formula to compute the steering angle.
        
