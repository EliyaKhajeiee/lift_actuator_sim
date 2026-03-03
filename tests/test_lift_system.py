"""
test suite for the lift actuator simulation system.

covers:
- light and heavy user scenarios
- overload conditions
- Emergency stop functionality
- stroke limit enforcement
- stable convergence
- overshoot tolerance
- fault injection scenarios
"""

import pytest
import numpy as np
import sys
import os

# add parent directory to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from actuator import LinearActuator
from load import UserLoad
from controller import LiftController
from simulation import LiftSimulation, FaultCondition


class TestActuatorPhysics:
    """test basic actuator physics and constraints"""

    def test_actuator_initialization(self):
        """make sure actuator initializes with correct default state"""
        actuator = LinearActuator(max_force=1500, stroke_length=0.5)

        assert actuator.position == 0.0
        assert actuator.velocity == 0.0
        assert actuator.acceleration == 0.0
        assert not actuator.emergency_stopped
        assert not actuator.stalled

    def test_stroke_limit_enforcement(self):
        """Test actuator cannot exceed stroke limits."""
        actuator = LinearActuator(max_force=1500, stroke_length=0.5)

        # try to move beyond upper limit
        actuator.set_pwm(1.0)
        for _ in range(1000):
            actuator.step(dt=0.01, required_force=100)

        assert actuator.position <= actuator.stroke_length
        assert actuator.position == actuator.stroke_length

    def test_force_limit_enforcement(self):
        """test actuator stalls when required force exceeds capacity"""
        actuator = LinearActuator(max_force=500, stroke_length=0.5)

        actuator.set_pwm(1.0)
        actuator.step(dt=0.01, required_force=1000)  # require more than max

        assert actuator.stalled
        assert actuator.velocity == 0.0

    def test_emergency_stop(self):
        """test that emergency stop immediately halts motion"""
        actuator = LinearActuator(max_force=1500, stroke_length=0.5)

        # get it moving
        actuator.set_pwm(1.0)
        for _ in range(10):
            actuator.step(dt=0.01, required_force=100)

        # hit the e-stop
        actuator.emergency_stop()

        assert actuator.emergency_stopped
        assert actuator.pwm_input == 0.0
        assert actuator.velocity == 0.0


class TestUserLoad:
    """test user load model"""

    def test_load_initialization(self):
        """make sure load initializes correctly"""
        load = UserLoad(mass=70)

        assert load.get_mass() == 70

    def test_force_calculation(self):
        """Test force calculation is correct."""
        load = UserLoad(mass=100)

        # at rest (no acceleration)
        force = load.get_required_force(acceleration=0.0)
        expected = 100 * 9.81
        assert abs(force - expected) < 0.01

        # with upward acceleration
        force = load.get_required_force(acceleration=2.0)
        expected = 100 * (9.81 + 2.0)
        assert abs(force - expected) < 0.01

    def test_load_shift(self):
        """test that dynamic load shift increases required force"""
        load = UserLoad(mass=100, load_shift_factor=0.5)

        base_force = load.get_required_force()
        load.apply_load_shift(1.0)
        shifted_force = load.get_required_force()

        assert shifted_force > base_force


class TestLiftController:
    """Test controller functionality."""

    def test_controller_initialization(self):
        """make sure controller initializes correctly"""
        actuator = LinearActuator(max_force=1500, stroke_length=0.5)
        load = UserLoad(mass=70)
        controller = LiftController(actuator, load)

        assert controller.target_position == 0.0
        assert not controller.overload_detected

    def test_target_validation(self):
        """test controller rejects invalid targets"""
        actuator = LinearActuator(max_force=1500, stroke_length=0.5)
        load = UserLoad(mass=70)
        controller = LiftController(actuator, load)

        # valid target
        assert controller.set_target_position(0.3)

        # invalid targets
        assert not controller.set_target_position(-0.1)
        assert not controller.set_target_position(1.0)

    def test_overload_detection(self):
        """test controller detects overload conditions"""
        actuator = LinearActuator(max_force=500, stroke_length=0.5)
        load = UserLoad(mass=200)  # requires ~1962N, way over 500N max
        controller = LiftController(actuator, load)

        controller.set_target_position(0.3)
        pwm = controller.update(dt=0.01)

        # should catch the overload
        assert controller.overload_detected


class TestLiftSimulation:
    """test complete lift simulation scenarios"""

    def test_light_user_lift(self):
        """Test successful lift with 60kg user."""
        actuator = LinearActuator(max_force=1500, stroke_length=0.5)
        load = UserLoad(mass=60)
        controller = LiftController(actuator, load, kp=3.0, ki=0.2, kd=0.8)
        sim = LiftSimulation(actuator, load, controller, dt=0.01)

        result = sim.run(target_height=0.3, max_time=40.0)

        # should succeed
        assert result.success, f"Failed: {result.failure_reason}"

        # should reach target within tolerance
        assert abs(result.position[-1] - 0.3) < 0.01

        # should be stable
        assert abs(result.velocity[-1]) < 0.01

        # should not violate safety limits
        summary = result.get_summary()
        assert summary['max_force'] < actuator.max_force

    def test_heavy_user_lift(self):
        """test successful lift with 140kg user"""
        actuator = LinearActuator(max_force=2500, stroke_length=0.5)
        load = UserLoad(mass=140)
        controller = LiftController(actuator, load, kp=3.0, ki=0.2, kd=0.8)
        sim = LiftSimulation(actuator, load, controller, dt=0.01)

        result = sim.run(target_height=0.3, max_time=40.0)

        # should succeed with higher force actuator
        assert result.success, f"Failed: {result.failure_reason}"
        assert abs(result.position[-1] - 0.3) < 0.01

    def test_overload_scenario(self):
        """Test system correctly detects overload condition."""
        actuator = LinearActuator(max_force=800, stroke_length=0.5)
        load = UserLoad(mass=150)  # needs ~1471N, way over 800N
        controller = LiftController(actuator, load, kp=3.0, ki=0.2, kd=0.8)
        sim = LiftSimulation(actuator, load, controller, dt=0.01)

        result = sim.run(target_height=0.3, max_time=40.0)

        # should fail with overload
        assert not result.success
        assert result.failure_reason == "Overload"

    def test_actuator_insufficient_force(self):
        """test actuator with insufficient force can't complete lift"""
        actuator = LinearActuator(max_force=500, stroke_length=0.5)
        load = UserLoad(mass=100)  # needs ~981N
        controller = LiftController(actuator, load)
        sim = LiftSimulation(actuator, load, controller, dt=0.01)

        result = sim.run(target_height=0.3, max_time=15.0)

        assert not result.success
        assert result.failure_reason in ["Overload", "Actuator stalled"]

    def test_emergency_stop_mid_lift(self):
        """test emergency stop halts lift mid-operation"""
        actuator = LinearActuator(max_force=1500, stroke_length=0.5)
        load = UserLoad(mass=70)
        controller = LiftController(actuator, load)
        sim = LiftSimulation(actuator, load, controller, dt=0.01)

        # add fault that triggers emergency stop at 2 seconds
        class EmergencyStopFault(FaultCondition):
            def __init__(self):
                super().__init__('emergency_stop', trigger_time=2.0)

        # monkey patch to trigger emergency stop
        original_process = sim._process_faults

        def custom_process(result):
            if sim.current_time >= 2.0 and not controller.emergency_stop_triggered:
                controller.emergency_stop()
                result.log_event(sim.current_time, 'fault', 'Emergency stop triggered')

        sim._process_faults = custom_process

        result = sim.run(target_height=0.3, max_time=15.0)

        # should fail due to emergency stop
        assert not result.success
        assert result.failure_reason == "Emergency stop"

        # should have stopped before reaching target
        assert result.position[-1] < 0.3

    def test_no_overshoot_beyond_tolerance(self):
        """make sure system doesn't overshoot target significantly"""
        actuator = LinearActuator(max_force=1500, stroke_length=0.5)
        load = UserLoad(mass=70)
        controller = LiftController(actuator, load, kp=2.5, kd=1.0)
        sim = LiftSimulation(actuator, load, controller, dt=0.01)

        result = sim.run(target_height=0.3, max_time=40.0)

        summary = result.get_summary()

        # allow small overshoot (< 5cm)
        assert summary['overshoot'] < 0.05

    def test_stable_convergence(self):
        """test system converges stably to target"""
        actuator = LinearActuator(max_force=1500, stroke_length=0.5)
        load = UserLoad(mass=70)
        controller = LiftController(actuator, load)
        sim = LiftSimulation(actuator, load, controller, dt=0.01)

        result = sim.run(target_height=0.3, max_time=40.0, require_stable=True)

        assert result.success

        # final velocity should be near zero
        assert abs(result.velocity[-1]) < 0.01

        # final position should be at target
        assert abs(result.position[-1] - 0.3) < 0.01

    def test_sudden_load_increase_fault(self):
        """test system response to sudden load increase"""
        actuator = LinearActuator(max_force=2000, stroke_length=0.5)
        load = UserLoad(mass=80)
        controller = LiftController(actuator, load, kp=3.0, ki=0.3, kd=0.8)
        sim = LiftSimulation(actuator, load, controller, dt=0.01)

        # inject load increase at 3 seconds
        fault = FaultCondition(
            fault_type='load_increase',
            trigger_time=3.0,
            severity=0.3  # 30% increase
        )
        sim.add_fault(fault)

        result = sim.run(target_height=0.3, max_time=40.0)

        # with adequate max force, should still succeed
        assert result.success

    def test_multiple_height_targets(self):
        """test system can reach multiple sequential targets"""
        actuator = LinearActuator(max_force=1500, stroke_length=0.5)
        load = UserLoad(mass=70)
        controller = LiftController(actuator, load)

        targets = [0.2, 0.35, 0.15]

        for target in targets:
            sim = LiftSimulation(actuator, load, controller, dt=0.01)
            result = sim.run(target_height=target, max_time=50.0)

            assert result.success, f"Failed to reach {target}m"
            assert abs(result.position[-1] - target) < 0.01


class TestRealismAndPhysics:
    """test physical realism of the simulation"""

    def test_acceleration_ramping(self):
        """Test actuator has smooth acceleration ramping."""
        actuator = LinearActuator(
            max_force=1500,
            stroke_length=0.5,
            acceleration_ramp_time=0.5
        )

        actuator.set_pwm(1.0)

        accelerations = []
        for _ in range(100):
            actuator.step(dt=0.01, required_force=100)
            accelerations.append(actuator.acceleration)

        # should ramp up smoothly, not jump instantly
        assert accelerations[0] < accelerations[50]

    def test_force_mass_relationship(self):
        """test heavier loads need proportionally more force"""
        mass_light = 50
        mass_heavy = 100

        load_light = UserLoad(mass=mass_light)
        load_heavy = UserLoad(mass=mass_heavy)

        force_light = load_light.get_required_force()
        force_heavy = load_heavy.get_required_force()

        # force should scale linearly with mass
        ratio = force_heavy / force_light
        expected_ratio = mass_heavy / mass_light

        assert abs(ratio - expected_ratio) < 0.01


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
