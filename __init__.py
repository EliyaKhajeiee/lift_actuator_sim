"""
Lift actuator simulation library

a Python library for simulating linear actuator lift systems.

Main components:
- LinearActuator: physical actuator sim with realistic dynamics
- UserLoad: load/user mass model
- LiftController: PID-based closed-loop controller
- LiftSimulation: complete sim engine with fault injection
"""

from .actuator import LinearActuator
from .load import UserLoad
from .controller import LiftController
from .simulation import LiftSimulation, FaultCondition, SimulationResult

__version__ = "0.1.0"
__all__ = [
    "LinearActuator",
    "UserLoad",
    "LiftController",
    "LiftSimulation",
    "FaultCondition",
    "SimulationResult"
]
