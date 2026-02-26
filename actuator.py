"""
Linear actuator physics simulation

models the actuator as a discrete-time system with realistic constraints.
based on basic physics: F=ma, velocity integration, position limits, etc.
"""

from typing import Optional
import numpy as np


class LinearActuator:
    """
    Simulates a linear actuator with realistic physics and constraints.

    models discrete time-step motion with acceleration ramping,
    force limits, and stroke constraints.
    """

    def __init__(
        self,
        max_force: float,
        stroke_length: float,
        max_velocity: float = 0.1,
        max_acceleration: float = 0.5,
        acceleration_ramp_time: float = 0.2
    ):
        """
        set up the linear actuator sim.

        Args:
            max_force: max force the actuator can push (Newtons)
            stroke_length: max extension length (meters)
            max_velocity: max linear velocity (m/s)
            max_acceleration: max acceleration (m/s²)
            acceleration_ramp_time: time to ramp up to full acceleration (seconds)
        """
        self.max_force = max_force
        self.stroke_length = stroke_length
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.acceleration_ramp_time = acceleration_ramp_time

        # state variables
        self.position = 0.0  # current position (m)
        self.velocity = 0.0  # current velocity (m/s)
        self.acceleration = 0.0  # current acceleration (m/s²)
        self.pwm_input = 0.0  # PWM input signal (0-1 range, signed for direction)
        self.emergency_stopped = False
        self.stalled = False

        # internal state
        self._target_acceleration = 0.0

    def set_pwm(self, pwm: float) -> None:
        """
        Set the PWM control signal.

        Args:
            pwm: PWM value in range [-1, 1] where:
                  1 = full extension
                 -1 = full retraction
                  0 = stopped
        """
        if self.emergency_stopped:
            self.pwm_input = 0.0
            return

        self.pwm_input = np.clip(pwm, -1.0, 1.0)

    def emergency_stop(self) -> None:
        """immediately stop the actuator and lock it out"""
        self.emergency_stopped = True
        self.pwm_input = 0.0
        self._target_acceleration = 0.0
        self.acceleration = 0.0
        self.velocity = 0.0

    def reset_emergency_stop(self) -> None:
        """clear the emergency stop condition"""
        self.emergency_stopped = False

    def can_apply_force(self, required_force: float) -> bool:
        """
        check if actuator can actually apply the required force.

        Args:
            required_force: force needed (Newtons)

        Returns:
            True if actuator can handle it, False otherwise
        """
        return abs(required_force) <= self.max_force

    def step(self, dt: float, required_force: float = 0.0) -> None:
        """
        Update actuator state for one time step.

        Args:
            dt: time step (seconds)
            required_force: force required to move the load (Newtons)
        """
        if self.emergency_stopped:
            self.velocity = 0.0
            self.acceleration = 0.0
            return

        # check if we can overcome the required force
        if not self.can_apply_force(required_force):
            self.stalled = True
            self.velocity = 0.0
            self.acceleration = 0.0
            return
        else:
            self.stalled = False

        # compute target acceleration from PWM with ramping
        self._target_acceleration = self.pwm_input * self.max_acceleration

        # ramp acceleration smoothly - motors can't change instantly
        ramp_rate = self.max_acceleration / self.acceleration_ramp_time
        accel_diff = self._target_acceleration - self.acceleration
        max_change = ramp_rate * dt

        if abs(accel_diff) <= max_change:
            self.acceleration = self._target_acceleration
        else:
            self.acceleration += np.sign(accel_diff) * max_change

        # integrate to get velocity
        self.velocity += self.acceleration * dt
        self.velocity = np.clip(self.velocity, -self.max_velocity, self.max_velocity)

        # damping near zero to stop oscillation - helps with stability
        if abs(self.pwm_input) < 0.05:
            self.velocity *= 0.9  # TODO: might need to tune this

        # update position
        new_position = self.position + self.velocity * dt

        # enforce stroke limits
        if new_position < 0.0:
            self.position = 0.0
            self.velocity = 0.0
            self.acceleration = 0.0
        elif new_position > self.stroke_length:
            self.position = self.stroke_length
            self.velocity = 0.0
            self.acceleration = 0.0
        else:
            self.position = new_position

    def get_state(self) -> dict:
        """
        get current actuator state.

        Returns:
            dict with position, velocity, acceleration, PWM, and flags
        """
        return {
            'position': self.position,
            'velocity': self.velocity,
            'acceleration': self.acceleration,
            'pwm': self.pwm_input,
            'emergency_stopped': self.emergency_stopped,
            'stalled': self.stalled
        }

    def reset(self) -> None:
        """reset actuator back to initial state"""
        self.position = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0
        self.pwm_input = 0.0
        self.emergency_stopped = False
        self.stalled = False
        self._target_acceleration = 0.0
