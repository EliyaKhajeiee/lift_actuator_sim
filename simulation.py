"""
Lift simulation engine module

runs complete lift scenarios with fault injection and telemetry logging
"""

from typing import Optional, Callable, List, Dict, Any
import numpy as np

try:
    from .actuator import LinearActuator
    from .load import UserLoad
    from .controller import LiftController
except ImportError:
    from actuator import LinearActuator
    from load import UserLoad
    from controller import LiftController


class FaultCondition:
    """represents a fault condition to inject during simulation"""

    def __init__(
        self,
        fault_type: str,
        trigger_time: float,
        duration: Optional[float] = None,
        severity: float = 1.0
    ):
        """
        set up a fault condition.

        Args:
            fault_type: type of fault ('sensor_dropout', 'load_increase',
                       'actuator_stall', 'position_noise')
            trigger_time: sim time to trigger the fault (seconds)
            duration: how long the fault lasts (None = permanent)
            severity: severity multiplier (0-1 range)
        """
        self.fault_type = fault_type
        self.trigger_time = trigger_time
        self.duration = duration
        self.severity = severity
        self.active = False
        self.start_time = 0.0

    def should_activate(self, current_time: float) -> bool:
        """check if this fault should kick in at the current time"""
        return current_time >= self.trigger_time and not self.active

    def should_deactivate(self, current_time: float) -> bool:
        """Check if fault should turn off at current time."""
        if not self.active or self.duration is None:
            return False
        return current_time >= (self.start_time + self.duration)


class SimulationResult:
    """container for sim results and telemetry data"""

    def __init__(self):
        self.time: List[float] = []
        self.position: List[float] = []
        self.velocity: List[float] = []
        self.acceleration: List[float] = []
        self.pwm: List[float] = []
        self.force: List[float] = []
        self.target_position: List[float] = []
        self.error: List[float] = []
        self.events: List[Dict[str, Any]] = []
        self.success = False
        self.failure_reason: Optional[str] = None

    def log_state(
        self,
        time: float,
        actuator_state: dict,
        control_state: dict,
        force: float
    ) -> None:
        """log one timestep worth of telemetry"""
        self.time.append(time)
        self.position.append(actuator_state['position'])
        self.velocity.append(actuator_state['velocity'])
        self.acceleration.append(actuator_state['acceleration'])
        self.pwm.append(actuator_state['pwm'])
        self.force.append(force)
        self.target_position.append(control_state['target_position'])
        self.error.append(control_state['error'])

    def log_event(self, time: float, event_type: str, description: str) -> None:
        """Log a sim event."""
        self.events.append({
            'time': time,
            'type': event_type,
            'description': description
        })

    def get_summary(self) -> dict:
        """get a summary of the sim results"""
        if len(self.position) == 0:
            return {'success': False, 'reason': 'No data logged'}

        return {
            'success': self.success,
            'failure_reason': self.failure_reason,
            'final_position': self.position[-1],
            'final_velocity': self.velocity[-1],
            'max_velocity': max(np.abs(self.velocity)),
            'max_acceleration': max(np.abs(self.acceleration)),
            'max_force': max(self.force),
            'settling_time': self._compute_settling_time(),
            'overshoot': self._compute_overshoot(),
            'num_events': len(self.events),
            'total_time': self.time[-1] if self.time else 0.0
        }

    def _compute_settling_time(self) -> Optional[float]:
        """figure out how long it takes to settle within tolerance"""
        tolerance = 0.01  # 1cm
        target = self.target_position[-1] if self.target_position else 0

        for i in range(len(self.position) - 1, -1, -1):
            if abs(self.position[i] - target) > tolerance:
                if i < len(self.time) - 1:
                    return self.time[i + 1]
                return None
        return self.time[0] if self.time else None

    def _compute_overshoot(self) -> float:
        """compute max overshoot past the target"""
        if not self.target_position or not self.position:
            return 0.0

        target = self.target_position[-1]
        max_pos = max(self.position)

        if max_pos > target:
            return max_pos - target
        return 0.0


class LiftSimulation:
    """
    Main simulation engine for running lift scenarios.

    ties together actuator, load, and controller with fault injection
    and telemetry logging
    """

    def __init__(
        self,
        actuator: LinearActuator,
        load: UserLoad,
        controller: LiftController,
        dt: float = 0.01
    ):
        """
        set up the simulation engine.

        Args:
            actuator: LinearActuator instance
            load: UserLoad instance
            controller: LiftController instance
            dt: sim timestep in seconds
        """
        self.actuator = actuator
        self.load = load
        self.controller = controller
        self.dt = dt

        self.faults: List[FaultCondition] = []
        self.current_time = 0.0

    def add_fault(self, fault: FaultCondition) -> None:
        """add a fault condition to the sim"""
        self.faults.append(fault)

    def reset(self) -> None:
        """Reset everything back to initial state."""
        self.actuator.reset()
        self.controller.reset()
        self.load.reset_load_shift()
        self.current_time = 0.0
        for fault in self.faults:
            fault.active = False

    def run(
        self,
        target_height: float,
        max_time: float = 30.0,
        require_stable: bool = True
    ) -> SimulationResult:
        """
        run a full lift simulation.

        Args:
            target_height: target lift height in meters
            max_time: max sim time (seconds)
            require_stable: if True, waits for stable convergence

        Returns:
            SimulationResult with all the telemetry and outcome
        """
        self.reset()
        result = SimulationResult()

        # set the target
        if not self.controller.set_target_position(target_height):
            result.failure_reason = "Invalid target height"
            return result

        result.log_event(0.0, 'start', f'Starting lift to {target_height}m')

        # main sim loop
        while self.current_time < max_time:
            # handle any faults
            self._process_faults(result)

            # figure out required force
            required_force = self.load.get_required_force(
                self.actuator.acceleration
            )

            # controller update
            pwm = self.controller.update(self.dt)

            # send PWM to actuator
            self.actuator.set_pwm(pwm)

            # step the actuator physics forward
            self.actuator.step(self.dt, required_force)

            # log telemetry
            result.log_state(
                self.current_time,
                self.actuator.get_state(),
                self.controller.get_control_state(),
                required_force
            )

            # check if something went wrong
            if self.controller.overload_detected:
                result.log_event(
                    self.current_time,
                    'failure',
                    'Overload detected - actuator cannot lift load'
                )
                result.failure_reason = "Overload"
                return result

            if self.actuator.stalled:
                result.log_event(
                    self.current_time,
                    'failure',
                    'Actuator stalled'
                )
                result.failure_reason = "Actuator stalled"
                return result

            if self.controller.emergency_stop_triggered:
                result.log_event(
                    self.current_time,
                    'failure',
                    'Emergency stop triggered'
                )
                result.failure_reason = "Emergency stop"
                return result

            # see if we've reached the target
            if require_stable:
                if self.controller.is_stable():
                    result.log_event(
                        self.current_time,
                        'success',
                        'Reached target and stabilized'
                    )
                    result.success = True
                    return result
            else:
                if self.controller.at_target():
                    result.log_event(
                        self.current_time,
                        'success',
                        'Reached target position'
                    )
                    result.success = True
                    return result

            # tick forward
            self.current_time += self.dt

        # ran out of time
        result.log_event(max_time, 'timeout', 'Simulation timeout')
        result.failure_reason = "Timeout"
        return result

    def _process_faults(self, result: SimulationResult) -> None:
        """go through faults and apply any that are due"""
        for fault in self.faults:
            # activate fault
            if fault.should_activate(self.current_time):
                fault.active = True
                fault.start_time = self.current_time
                result.log_event(
                    self.current_time,
                    'fault',
                    f'Fault activated: {fault.fault_type}'
                )
                self._apply_fault(fault)

            # deactivate fault if its time is up
            if fault.should_deactivate(self.current_time):
                fault.active = False
                result.log_event(
                    self.current_time,
                    'fault_clear',
                    f'Fault cleared: {fault.fault_type}'
                )
                self._clear_fault(fault)

    def _apply_fault(self, fault: FaultCondition) -> None:
        """Apply a fault to the system."""
        if fault.fault_type == 'sensor_dropout':
            # would need position feedback sim for this one
            pass

        elif fault.fault_type == 'load_increase':
            # crank up the load mass suddenly
            original_mass = self.load.get_mass()
            new_mass = original_mass * (1.0 + fault.severity)
            self.load.set_mass(new_mass)

        elif fault.fault_type == 'actuator_stall':
            # force the actuator into a stall
            self.actuator.stalled = True

        elif fault.fault_type == 'load_shift':
            # apply a dynamic load shift
            self.load.apply_load_shift(fault.severity)

    def _clear_fault(self, fault: FaultCondition) -> None:
        """Clear a fault from the system."""
        if fault.fault_type == 'load_shift':
            self.load.reset_load_shift()
        elif fault.fault_type == 'actuator_stall':
            self.actuator.stalled = False
