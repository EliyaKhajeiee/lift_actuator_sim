"""
User load model module

Models the user/load being lifted by the actuator system.
"""

from typing import Optional
import numpy as np


class UserLoad:
    """
    Models a user load with mass and optional dynamic load shifts.

    computes the required force to lift the load considering:
    - gravitational force
    - acceleration forces
    - dynamic load shift factors
    """

    GRAVITY = 9.81  # m/s²

    def __init__(
        self,
        mass: float,
        load_shift_factor: float = 0.0
    ):
        """
        set up the user load model.

        Args:
            mass: mass of the user/load (kg)
            load_shift_factor: dynamic load shift multiplier (0-1 range)
                              simulates user movement or load redistribution
        """
        self.mass = mass
        self.load_shift_factor = load_shift_factor
        self._current_shift = 0.0

    def get_required_force(self, acceleration: float = 0.0) -> float:
        """
        calculate the force needed to lift the load.

        Uses F = m(g + a) where:
        - m = mass
        - g = gravitational acceleration
        - a = system acceleration

        Args:
            acceleration: current acceleration of the system (m/s²)

        Returns:
            required force in Newtons
        """
        # base force: F = m * g
        base_force = self.mass * self.GRAVITY

        # acceleration force: F = m * a
        accel_force = self.mass * acceleration

        # dynamic load shift (simulates user moving around)
        shift_force = base_force * self._current_shift * self.load_shift_factor

        total_force = base_force + accel_force + shift_force
        return total_force

    def apply_load_shift(self, shift: float) -> None:
        """
        apply a dynamic load shift.

        Args:
            shift: shift factor in range [-1, 1]
                   positive = increased effective load
                   negative = decreased effective load
        """
        self._current_shift = np.clip(shift, -1.0, 1.0)

    def reset_load_shift(self) -> None:
        """reset any dynamic load shift back to zero"""
        self._current_shift = 0.0

    def get_mass(self) -> float:
        """get the load mass"""
        return self.mass

    def set_mass(self, mass: float) -> None:
        """
        set a new load mass.

        Args:
            mass: new mass value (kg)
        """
        if mass < 0:
            raise ValueError("Mass cannot be negative")
        self.mass = mass
