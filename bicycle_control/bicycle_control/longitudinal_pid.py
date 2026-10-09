"""
Low-Level Powertrain Cruise Controller (Longitudinal PID).
Regulates vehicle speed via normalized throttle/braking effort.
"""

import numpy as np  # noqa: F401


class PIDLongitudinalController:
    """Low-Level Powertrain Cruise Controller / Electronic Speed Control (ESC).

    Translates high-level velocity requests into normalized throttle/brake effort.
    Because physical vehicles experience friction and speed-squared aerodynamic drag,
    a closed-loop speed regulator is required to maintain target velocity.
    """

    def __init__(self, kp=1.0, ki=0.2, kd=0.05, dt=0.1,
                 max_throttle=1.0, max_brake=1.0, integral_limit=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.integral_limit = integral_limit

        self.integral = 0.0
        self.prev_error = None

    def compute(self, target_vel, current_vel):
        """Computes normalized throttle/braking effort in [-1.0, 1.0]."""
        error = target_vel - current_vel
        p_term = self.kp * error
        self.integral += error*self.dt
        ## we beed to make the anti-windup on the integrator 
        self.integral = float(np.clip(self.integral , -self.integral_limit , self.integral_limit)) 
        i_term = self.ki * self.integral
        if self.prev_error is None:
           error_derv = 0.0
        else:
            error_derv = (error - self.prev_error) / self.dt
        d_term = self.kd * error_derv

        self.prev_error = error

        output = p_term + i_term + d_term
        ## max brake is positive number since it's just in the opposite direction of the throttle but the reason we added negative is because np.clip requires a lower boundary
        pid_output = float(np.clip(output , - self.max_brake , self.max_throttle))
        
        return pid_output

        # TODO: Milestone 4.1 — Longitudinal PID Speed Control & Anti-Windup
        # This is the speed regulator. Because the car has drag, simply setting
        # a target speed isn't enough — it needs closed-loop control.
        # Implement a PID controller on the velocity error with anti-windup on the integrator.
    

    def reset(self):
        """Resets integrator and previous error state."""
        self.integral = 0.0
        self.prev_error = None
