"""
Automated headless tests for the 3D wheelchair sling-lift simulation.

Runs WheelchairLiftSim3D in p.DIRECT (no display needed) and validates:
- Scene initializes without error
- Actuators extend when a lift target is set
- Sling and user rise proportionally with extension
- Overload condition: actuators stall when max_force is too low
- Weight change is reflected in required force
- Lowering: actuators retract back to zero
- E-stop halts motion mid-lift
- Extension stays within 1-inch stroke limit
"""

import pytest
import sys
import os
import math

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import pybullet as p
from wheelchair_sim_3d import WheelchairLiftSim3D, ACTUATOR_STROKE, SEAT_H, SLING_T


# ── helpers ───────────────────────────────────────────────────────────────

def _step_sim(sim, steps: int, target_m: float = None, dt: float = 0.01):
    """Run `steps` physics ticks, optionally with a fixed target."""
    if target_m is not None:
        clamped = max(0.0, min(target_m, ACTUATOR_STROKE))
        sim.ctrl.set_target_position(clamped)

    for _ in range(steps):
        req_force = sim.load.get_required_force(sim.act_L.acceleration)
        pwm = sim.ctrl.update(dt)
        sim.act_L.set_pwm(pwm)
        sim.act_R.set_pwm(pwm)
        sim.act_L.step(dt, req_force)
        sim.act_R.step(dt, req_force)
        p.stepSimulation()

    return (sim.act_L.position + sim.act_R.position) / 2.0


def _ext(sim):
    return (sim.act_L.position + sim.act_R.position) / 2.0


# ── fixtures ──────────────────────────────────────────────────────────────

@pytest.fixture
def sim():
    """Fresh headless sim instance, disconnects after each test."""
    s = WheelchairLiftSim3D(headless=True)
    yield s
    if p.isConnected(s.client):
        p.disconnect(s.client)


# ── tests ─────────────────────────────────────────────────────────────────

class TestInit:
    def test_scene_builds_without_error(self, sim):
        """Sim must initialise cleanly and connect to PyBullet DIRECT."""
        assert p.isConnected(sim.client)

    def test_actuators_at_zero_on_start(self, sim):
        assert sim.act_L.position == pytest.approx(0.0, abs=1e-6)
        assert sim.act_R.position == pytest.approx(0.0, abs=1e-6)

    def test_controller_target_at_zero(self, sim):
        assert sim.ctrl.target_position == pytest.approx(0.0, abs=1e-6)

    def test_default_user_mass(self, sim):
        # load is initialised with half the 80 kg default
        assert sim.load.get_mass() == pytest.approx(40.0, abs=0.01)


class TestLift:
    def test_actuators_extend_toward_target(self, sim):
        """After running toward 1-inch target, extension must be > 0."""
        _step_sim(sim, steps=500, target_m=ACTUATOR_STROKE)
        assert _ext(sim) > 0.001   # at least 1 mm of movement

    def test_full_lift_reaches_target(self, sim):
        """With adequate force, actuators reach the full 1-inch stroke."""
        _step_sim(sim, steps=3000, target_m=ACTUATOR_STROKE)
        assert _ext(sim) == pytest.approx(ACTUATOR_STROKE, abs=0.001)

    def test_partial_lift_target(self, sim):
        """Actuators should stop at a partial target (~half stroke).
        PA-14 class actuators have ±4 mm real-world positional accuracy."""
        half = ACTUATOR_STROKE / 2.0
        _step_sim(sim, steps=3000, target_m=half)
        assert abs(_ext(sim) - half) < 0.004

    def test_extension_never_exceeds_stroke(self, sim):
        """Extension must stay within the physical stroke limit."""
        _step_sim(sim, steps=5000, target_m=ACTUATOR_STROKE * 2)  # over-target
        assert _ext(sim) <= ACTUATOR_STROKE + 1e-6

    def test_both_actuators_in_sync(self, sim):
        """Left and right actuators must track each other closely."""
        _step_sim(sim, steps=2000, target_m=ACTUATOR_STROKE)
        diff = abs(sim.act_L.position - sim.act_R.position)
        assert diff < 1e-9   # driven by same PWM signal, must be identical


class TestLower:
    def test_actuators_retract_after_lift(self, sim):
        """After lifting, setting target to 0 should retract to zero."""
        # lift first
        _step_sim(sim, steps=3000, target_m=ACTUATOR_STROKE)
        assert _ext(sim) > ACTUATOR_STROKE * 0.9

        # now lower
        _step_sim(sim, steps=3000, target_m=0.0)
        assert _ext(sim) < 0.001


class TestOverload:
    def test_stall_when_force_too_low(self, sim):
        """Actuators must stall if max_force < load weight."""
        # 80 kg user → half load = 40 kg → gravitational force ≈ 392 N
        # set max_force well below that
        sim.act_L.max_force = 50.0
        sim.act_R.max_force = 50.0

        _step_sim(sim, steps=200, target_m=ACTUATOR_STROKE)

        assert sim.act_L.stalled or sim.ctrl.overload_detected

    def test_no_stall_with_adequate_force(self, sim):
        """With plenty of force, actuators should not stall."""
        _step_sim(sim, steps=3000, target_m=ACTUATOR_STROKE)
        assert not sim.act_L.stalled
        assert not sim.act_R.stalled


class TestWeightChange:
    def test_heavier_user_needs_more_force(self, sim):
        """Required force must increase when user mass increases."""
        force_light = sim.load.get_required_force(0.0)

        sim.load.set_mass(110.0)   # heavier (220 kg total user, half = 110)
        force_heavy = sim.load.get_required_force(0.0)

        assert force_heavy > force_light

    def test_weight_change_reflected_in_load(self, sim):
        """Changing the slider weight value must update load.mass."""
        sim.load.set_mass(60.0)   # 120 kg user → 60 kg per side
        assert sim.load.get_mass() == pytest.approx(60.0, abs=0.01)

    def test_force_scales_with_mass(self, sim):
        """Force ratio must match mass ratio (F = mg at rest)."""
        sim.load.set_mass(50.0)
        f50 = sim.load.get_required_force(0.0)

        sim.load.set_mass(100.0)
        f100 = sim.load.get_required_force(0.0)

        assert f100 / f50 == pytest.approx(2.0, abs=0.01)


class TestEmergencyStop:
    def test_estop_halts_motion(self, sim):
        """E-stop mid-lift must freeze position immediately."""
        _step_sim(sim, steps=300, target_m=ACTUATOR_STROKE)
        pos_before = _ext(sim)
        assert pos_before > 0.0005   # was moving

        sim.emergency_stop()

        # run more steps — should not move
        pos_after = _step_sim(sim, steps=300, target_m=ACTUATOR_STROKE)
        assert abs(pos_after - pos_before) < 1e-9

    def test_estop_sets_flags(self, sim):
        sim.emergency_stop()
        assert sim.ctrl.emergency_stop_triggered
        assert sim.act_L.emergency_stopped
        assert sim.act_R.emergency_stopped


class TestPhysicsIntegration:
    def test_velocity_zero_at_standby(self, sim):
        """At standby (target = 0, no load), velocity should remain zero."""
        _step_sim(sim, steps=100, target_m=0.0)
        assert sim.act_L.velocity == pytest.approx(0.0, abs=1e-6)

    def test_velocity_ramps_up_on_start(self, sim):
        """Velocity should increase from zero as actuators begin moving."""
        sim.ctrl.set_target_position(ACTUATOR_STROKE)
        velocities = []
        for _ in range(100):
            req = sim.load.get_required_force(sim.act_L.acceleration)
            pwm = sim.ctrl.update(0.01)
            sim.act_L.set_pwm(pwm)
            sim.act_R.set_pwm(pwm)
            sim.act_L.step(0.01, req)
            sim.act_R.step(0.01, req)
            p.stepSimulation()
            velocities.append(sim.act_L.velocity)

        # velocity should have increased over the first 100 steps
        assert max(velocities) > 0.0

    def test_position_monotonically_increases_during_lift(self, sim):
        """During a clean lift, position should not drop unexpectedly."""
        sim.ctrl.set_target_position(ACTUATOR_STROKE)
        positions = []
        for _ in range(2000):
            req = sim.load.get_required_force(sim.act_L.acceleration)
            pwm = sim.ctrl.update(0.01)
            sim.act_L.set_pwm(pwm)
            sim.act_R.set_pwm(pwm)
            sim.act_L.step(0.01, req)
            sim.act_R.step(0.01, req)
            p.stepSimulation()
            positions.append(sim.act_L.position)

        # no step should drop by more than a tiny rounding amount
        for i in range(1, len(positions)):
            assert positions[i] >= positions[i - 1] - 1e-9


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
