"""
Lift controller module

implements closed-loop control for the actuator lift system using PID control.
"""

from typing import Optional
import numpy as np

try:
    from .actuator import LinearActuator
    from .load import UserLoad
except ImportError:
    from actuator import LinearActuator
    from load import UserLoad


class LiftController:
    """
    PID controller for closed-loop lift control.

    handles target height tracking, safety constraints, and overload detection.
    """

    def __init__(
        self,
        actuator: LinearActuator,
        load: UserLoad,
        kp: float = 2.0,
        ki: float = 0.1,
        kd: float = 0.5,
        position_tolerance: float = 0.01
    ):
        """
        Initialize the lift controller.

        Args:
            actuator: LinearActuator instance to control
            load: UserLoad instance for the load
            kp: proportional gain
            ki: integral gain
            kd: derivative gain
            position_tolerance: acceptable position error (meters)
        """
        self.actuator = actuator
        self.load = load

        # PID gains
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # control state
        self.target_position = 0.0
        self.position_tolerance = position_tolerance
        self.integral_error = 0.0
        self.last_error = 0.0

        # safety state
        self.overload_detected = False
        self.emergency_stop_triggered = False

        # anti-windup - prevent integral from getting too large
        self.integral_limit = 10.0  # empirically chosen, seems to work

    def set_target_position(self, target: float) -> bool:
        """
        set the target lift height.

        Args:
            target: desired position (meters)

        Returns:
            True if target is valid, False otherwise
        """
        if target < 0 or target > self.actuator.stroke_length:
            return False

        self.target_position = target
        self.integral_error = 0.0  # reset integral on new target
        return True

    def update(self, dt: float) -> float:
        """
        run one control loop update.

        Args:
            dt: time step (seconds)

        Returns:
            computed PWM output value
        """
        if self.emergency_stop_triggered:
            return 0.0

        # grab current position
        current_position = self.actuator.position

        # compute error
        error = self.target_position - current_position

        # proportional term
        p_term = self.kp * error

        # integral term with anti-windup
        self.integral_error += error * dt
        self.integral_error = np.clip(
            self.integral_error,
            -self.integral_limit,
            self.integral_limit
        )
        i_term = self.ki * self.integral_error

        # derivative term
        if dt > 0:
            d_term = self.kd * (error - self.last_error) / dt
        else:
            d_term = 0.0

        # compute PWM output
        pwm_output = p_term + i_term + d_term

        # clamp to valid range
        pwm_output = np.clip(pwm_output, -1.0, 1.0)

        # check for overload
        required_force = self.load.get_required_force(self.actuator.acceleration)
        if not self.actuator.can_apply_force(required_force):
            self.overload_detected = True
            pwm_output = 0.0

        # store last error for derivative calc
        self.last_error = error

        return pwm_output

    def at_target(self) -> bool:
        """
        check if we've reached the target position.

        Returns:
            True if within tolerance of target
        """
        error = abs(self.target_position - self.actuator.position)
        return error <= self.position_tolerance

    def is_stable(self, velocity_threshold: float = 0.001) -> bool:
        """
        Check if the system is stable at the target.

        Args:
            velocity_threshold: max velocity to consider stable (m/s)

        Returns:
            True if at target and velocity is basically zero
        """
        return (
            self.at_target() and
            abs(self.actuator.velocity) <= velocity_threshold
        )

    def emergency_stop(self) -> None:
        """trigger emergency stop on the whole system"""
        self.emergency_stop_triggered = True
        self.actuator.emergency_stop()

    def reset_emergency_stop(self) -> None:
        """Clear emergency stop condition."""
        self.emergency_stop_triggered = False
        self.actuator.reset_emergency_stop()

    def reset(self) -> None:
        """reset controller state"""
        self.target_position = 0.0
        self.integral_error = 0.0
        self.last_error = 0.0
        self.overload_detected = False
        self.emergency_stop_triggered = False

    def get_control_state(self) -> dict:
        """
        Get current controller state.

        Returns:
            dict with controller state info
        """
        return {
            'target_position': self.target_position,
            'current_position': self.actuator.position,
            'error': self.target_position - self.actuator.position,
            'at_target': self.at_target(),
            'is_stable': self.is_stable(),
            'overload_detected': self.overload_detected,
            'emergency_stopped': self.emergency_stop_triggered,
            'integral_error': self.integral_error
        }
