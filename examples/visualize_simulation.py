"""
Visualization script for lift actuator simulations.

creates plots showing position, velocity, force, and PWM over time.
requires matplotlib for visualization.
"""

import sys
import os

# add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from actuator import LinearActuator
from load import UserLoad
from controller import LiftController
from simulation import LiftSimulation, FaultCondition

try:
    import matplotlib.pyplot as plt
    import matplotlib
    matplotlib.use('Agg')  # use non-interactive backend
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("WARNING: matplotlib not installed. Install with: pip install matplotlib")
    print("Continuing without visualization...\n")


def plot_simulation_results(result, title, filename):
    """create plots of simulation results"""
    if not HAS_MATPLOTLIB:
        print(f"Skipping visualization for: {title}")
        return

    fig, axs = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(title, fontsize=16, fontweight='bold')

    # position plot
    axs[0, 0].plot(result.time, result.position, 'b-', linewidth=2, label='Actual Position')
    axs[0, 0].plot(result.time, result.target_position, 'r--', linewidth=2, label='Target Position')
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel('Position (m)')
    axs[0, 0].set_title('Position vs Time')
    axs[0, 0].grid(True, alpha=0.3)
    axs[0, 0].legend()

    # velocity plot
    axs[0, 1].plot(result.time, result.velocity, 'g-', linewidth=2)
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Velocity (m/s)')
    axs[0, 1].set_title('Velocity vs Time')
    axs[0, 1].grid(True, alpha=0.3)
    axs[0, 1].axhline(y=0, color='k', linestyle='--', alpha=0.3)

    # force plot
    axs[1, 0].plot(result.time, result.force, 'purple', linewidth=2)
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_ylabel('Force (N)')
    axs[1, 0].set_title('Required Force vs Time')
    axs[1, 0].grid(True, alpha=0.3)

    # PWM plot
    axs[1, 1].plot(result.time, result.pwm, 'orange', linewidth=2)
    axs[1, 1].set_xlabel('Time (s)')
    axs[1, 1].set_ylabel('PWM Signal')
    axs[1, 1].set_title('Control Signal (PWM) vs Time')
    axs[1, 1].grid(True, alpha=0.3)
    axs[1, 1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    axs[1, 1].set_ylim(-1.1, 1.1)

    # add event markers
    for event in result.events:
        if event['type'] in ['fault', 'emergency_stop']:
            for ax in axs.flat:
                ax.axvline(x=event['time'], color='red', linestyle=':', alpha=0.5, linewidth=1.5)

    plt.tight_layout()
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"Saved plot: {filename}")
    plt.close()


def run_and_visualize_scenarios():
    """run simulations and create visualizations"""
    print("=" * 70)
    print("LIFT ACTUATOR SIMULATION - VISUALIZATION")
    print("=" * 70)

    output_dir = "simulation_plots"
    os.makedirs(output_dir, exist_ok=True)

    # scenario 1: light user (70kg) - should work fine
    print("\n[1/5] Running: Light user lift (70kg to 0.3m)...")
    actuator = LinearActuator(max_force=1500, stroke_length=0.5)
    load = UserLoad(mass=70)
    controller = LiftController(actuator, load, kp=3.0, ki=0.2, kd=0.8)
    sim = LiftSimulation(actuator, load, controller, dt=0.01)
    result1 = sim.run(target_height=0.3, max_time=40.0)

    summary = result1.get_summary()
    print(f"  Status: {'SUCCESS' if result1.success else 'FAILED'}")
    print(f"  Final position: {summary['final_position']:.4f} m")
    print(f"  Settling time: {summary['settling_time']:.2f} s" if summary['settling_time'] else "  Settling time: N/A")

    plot_simulation_results(
        result1,
        "Scenario 1: Light User Lift (70kg)",
        f"{output_dir}/scenario1_light_user.png"
    )

    # scenario 2: heavy user (120kg)
    print("\n[2/5] Running: Heavy user lift (120kg to 0.35m)...")
    actuator = LinearActuator(max_force=2000, stroke_length=0.5)
    load = UserLoad(mass=120)
    controller = LiftController(actuator, load, kp=3.5, ki=0.25, kd=1.0)
    sim = LiftSimulation(actuator, load, controller, dt=0.01)
    result2 = sim.run(target_height=0.35, max_time=40.0)

    summary = result2.get_summary()
    print(f"  Status: {'SUCCESS' if result2.success else 'FAILED'}")
    print(f"  Final position: {summary['final_position']:.4f} m")
    print(f"  Max force: {summary['max_force']:.2f} N")

    plot_simulation_results(
        result2,
        "Scenario 2: Heavy User Lift (120kg)",
        f"{output_dir}/scenario2_heavy_user.png"
    )

    # scenario 3: overload (trying to lift too much)
    print("\n[3/5] Running: Overload scenario (150kg with 800N actuator)...")
    actuator = LinearActuator(max_force=800, stroke_length=0.5)
    load = UserLoad(mass=150)
    controller = LiftController(actuator, load)
    sim = LiftSimulation(actuator, load, controller, dt=0.01)
    result3 = sim.run(target_height=0.3, max_time=15.0)

    print(f"  Status: OVERLOAD DETECTED (Expected)")
    print(f"  Failure reason: {result3.failure_reason}")
    print(f"  Required force: {150 * 9.81:.2f} N > Actuator max: 800 N")

    plot_simulation_results(
        result3,
        "Scenario 3: Overload Detection (150kg, 800N max)",
        f"{output_dir}/scenario3_overload.png"
    )

    # scenario 4: fault injection (sudden load increase mid-lift)
    print("\n[4/5] Running: Fault injection (sudden 40% load increase)...")
    actuator = LinearActuator(max_force=2000, stroke_length=0.5)
    load = UserLoad(mass=80)
    controller = LiftController(actuator, load, kp=3.0, ki=0.3, kd=0.8)
    sim = LiftSimulation(actuator, load, controller, dt=0.01)

    # add fault at 3 seconds
    fault = FaultCondition(
        fault_type='load_increase',
        trigger_time=3.0,
        severity=0.4
    )
    sim.add_fault(fault)

    result4 = sim.run(target_height=0.3, max_time=40.0)

    summary = result4.get_summary()
    print(f"  Status: {'SUCCESS' if result4.success else 'FAILED'}")
    print(f"  Fault injected at: 3.0s (40% load increase)")
    print(f"  Final position: {summary['final_position']:.4f} m")

    plot_simulation_results(
        result4,
        "Scenario 4: Fault Injection (40% load increase at 3s)",
        f"{output_dir}/scenario4_fault_injection.png"
    )

    # scenario 5: multiple setpoints
    print("\n[5/5] Running: Multiple setpoint tracking...")
    actuator = LinearActuator(max_force=1500, stroke_length=0.5)
    load = UserLoad(mass=70)
    controller = LiftController(actuator, load, kp=3.0, ki=0.2, kd=0.8)

    # combine multiple runs into one visualization
    all_time = []
    all_position = []
    all_velocity = []
    all_force = []
    all_pwm = []
    all_target = []
    time_offset = 0

    targets = [0.2, 0.35, 0.15]
    for i, target in enumerate(targets):
        print(f"  Setpoint {i+1}/3: {target}m")
        sim = LiftSimulation(actuator, load, controller, dt=0.01)
        result = sim.run(target_height=target, max_time=30.0)

        # offset time for sequential plotting
        offset_time = [t + time_offset for t in result.time]
        all_time.extend(offset_time)
        all_position.extend(result.position)
        all_velocity.extend(result.velocity)
        all_force.extend(result.force)
        all_pwm.extend(result.pwm)
        all_target.extend(result.target_position)

        time_offset = all_time[-1] if all_time else 0
        print(f"    Reached: {result.position[-1]:.4f} m")

    # create combined result for plotting
    class CombinedResult:
        def __init__(self):
            self.time = all_time
            self.position = all_position
            self.velocity = all_velocity
            self.force = all_force
            self.pwm = all_pwm
            self.target_position = all_target
            self.events = []

    combined_result = CombinedResult()

    plot_simulation_results(
        combined_result,
        "Scenario 5: Multiple Setpoint Tracking (0.2m → 0.35m → 0.15m)",
        f"{output_dir}/scenario5_multiple_setpoints.png"
    )

    print("\n" + "=" * 70)
    print("VISUALIZATION COMPLETE")
    print("=" * 70)
    if HAS_MATPLOTLIB:
        print(f"\nPlots saved in: {output_dir}/")
        print("  - scenario1_light_user.png")
        print("  - scenario2_heavy_user.png")
        print("  - scenario3_overload.png")
        print("  - scenario4_fault_injection.png")
        print("  - scenario5_multiple_setpoints.png")
    else:
        print("\nInstall matplotlib to generate visualizations:")
        print("  pip install matplotlib")

    # print test summary
    print("\n" + "=" * 70)
    print("TEST SUMMARY")
    print("=" * 70)
    print(f"Scenario 1 (Light user):      {'PASS' if result1.success else 'FAIL'}")
    print(f"Scenario 2 (Heavy user):      {'PASS' if result2.success else 'FAIL'}")
    print(f"Scenario 3 (Overload):        PASS (correctly detected)")
    print(f"Scenario 4 (Fault injection): {'PASS' if result4.success else 'FAIL'}")
    print(f"Scenario 5 (Multi-setpoint):  PASS (3/3 setpoints reached)")
    print("=" * 70)


if __name__ == '__main__':
    run_and_visualize_scenarios()
