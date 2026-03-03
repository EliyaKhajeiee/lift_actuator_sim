"""
Example simulation showing lift actuator system usage.

this script demos:
1. basic lift scenario
2. fault injection
3. telemetry analysis
4. multiple user weights
"""

import sys
import os

# add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from actuator import LinearActuator
from load import UserLoad
from controller import LiftController
from simulation import LiftSimulation, FaultCondition


def run_basic_lift():
    """run a basic lift scenario with a 70kg user"""
    print("=" * 60)
    print("SCENARIO 1: Basic Lift (70kg user to 0.3m)")
    print("=" * 60)

    # create system components
    actuator = LinearActuator(
        max_force=1500,      # 1500N max force
        stroke_length=0.5,   # 0.5m stroke
        max_velocity=0.1,    # 0.1 m/s max speed
    )

    load = UserLoad(mass=70)  # 70kg user

    controller = LiftController(
        actuator=actuator,
        load=load,
        kp=3.0,  # proportional gain
        ki=0.2,  # integral gain
        kd=0.8,  # derivative gain
    )

    sim = LiftSimulation(
        actuator=actuator,
        load=load,
        controller=controller,
        dt=0.01  # 10ms timestep
    )

    # run it
    result = sim.run(target_height=0.3, max_time=30.0)

    # print results
    print(f"\nResult: {'SUCCESS' if result.success else 'FAILURE'}")
    if not result.success:
        print(f"Failure reason: {result.failure_reason}")

    summary = result.get_summary()
    print(f"\nFinal position: {summary['final_position']:.4f} m")
    print(f"Final velocity: {summary['final_velocity']:.6f} m/s")
    print(f"Max velocity: {summary['max_velocity']:.4f} m/s")
    print(f"Max force: {summary['max_force']:.2f} N")
    settling_str = f"{summary['settling_time']:.2f} s" if summary['settling_time'] is not None else "N/A (not settled)"
    print(f"Settling time: {settling_str}")
    print(f"Overshoot: {summary['overshoot']:.4f} m")
    print(f"Total time: {summary['total_time']:.2f} s")

    print(f"\nEvents ({len(result.events)}):")
    for event in result.events:
        print(f"  {event['time']:.2f}s - {event['type']}: {event['description']}")

    return result


def run_heavy_user_lift():
    """run lift with a heavy user (140kg)"""
    print("\n" + "=" * 60)
    print("SCENARIO 2: Heavy User Lift (140kg to 0.35m)")
    print("=" * 60)

    actuator = LinearActuator(max_force=2500, stroke_length=0.5)
    load = UserLoad(mass=140)
    controller = LiftController(actuator, load, kp=3.5, ki=0.25, kd=1.0)
    sim = LiftSimulation(actuator, load, controller, dt=0.01)

    result = sim.run(target_height=0.35, max_time=20.0)

    print(f"\nResult: {'SUCCESS' if result.success else 'FAILURE'}")
    if not result.success:
        print(f"Failure reason: {result.failure_reason}")

    summary = result.get_summary()
    print(f"Final position: {summary['final_position']:.4f} m")
    print(f"Max force: {summary['max_force']:.2f} N")
    settling_str = f"{summary['settling_time']:.2f} s" if summary['settling_time'] is not None else "N/A (not settled)"
    print(f"Settling time: {settling_str}")

    return result


def run_overload_scenario():
    """demonstrate overload detection"""
    print("\n" + "=" * 60)
    print("SCENARIO 3: Overload Scenario (150kg with 800N actuator)")
    print("=" * 60)

    actuator = LinearActuator(max_force=800, stroke_length=0.5)
    load = UserLoad(mass=150)  # way too heavy for this actuator
    controller = LiftController(actuator, load)
    sim = LiftSimulation(actuator, load, controller, dt=0.01)

    result = sim.run(target_height=0.3, max_time=15.0)

    print(f"\nResult: {'SUCCESS' if result.success else 'FAILURE (Expected)'}")
    print(f"Failure reason: {result.failure_reason}")

    required_force = 150 * 9.81
    print(f"\nRequired force: {required_force:.2f} N")
    print(f"Actuator max force: 800 N")
    print("Actuator can't lift this load - overload correctly detected!")

    return result


def run_fault_injection():
    """demonstrate fault injection during lift"""
    print("\n" + "=" * 60)
    print("SCENARIO 4: Fault Injection (Sudden load increase)")
    print("=" * 60)

    actuator = LinearActuator(max_force=2000, stroke_length=0.5)
    load = UserLoad(mass=80)
    controller = LiftController(actuator, load, kp=3.0, ki=0.3, kd=0.8)
    sim = LiftSimulation(actuator, load, controller, dt=0.01)

    # inject fault: 40% load increase at 3 seconds
    fault = FaultCondition(
        fault_type='load_increase',
        trigger_time=3.0,
        severity=0.4
    )
    sim.add_fault(fault)

    print("\nFault configured:")
    print(f"  Type: {fault.fault_type}")
    print(f"  Trigger time: {fault.trigger_time}s")
    print(f"  Severity: {fault.severity * 100}%")

    result = sim.run(target_height=0.3, max_time=25.0)

    print(f"\nResult: {'SUCCESS' if result.success else 'FAILURE'}")
    summary = result.get_summary()
    print(f"Final position: {summary['final_position']:.4f} m")
    settling_str = f"{summary['settling_time']:.2f} s" if summary['settling_time'] is not None else "N/A (not settled)"
    print(f"Settling time: {settling_str}")

    print(f"\nEvents:")
    for event in result.events:
        print(f"  {event['time']:.2f}s - {event['type']}: {event['description']}")

    return result


def run_comparison_study():
    """compare performance across different user weights"""
    print("\n" + "=" * 60)
    print("SCENARIO 5: Weight Comparison Study")
    print("=" * 60)

    user_weights = [50, 70, 90, 110, 130]
    target = 0.3

    print(f"\nTarget height: {target}m")
    print(f"Actuator: 2000N max force, 0.5m stroke\n")

    results_table = []

    for weight in user_weights:
        actuator = LinearActuator(max_force=2000, stroke_length=0.5)
        load = UserLoad(mass=weight)
        controller = LiftController(actuator, load, kp=3.0, ki=0.2, kd=0.8)
        sim = LiftSimulation(actuator, load, controller, dt=0.01)

        result = sim.run(target_height=target, max_time=20.0)
        summary = result.get_summary()

        results_table.append({
            'weight': weight,
            'success': result.success,
            'settling_time': summary['settling_time'],
            'max_force': summary['max_force'],
            'overshoot': summary['overshoot']
        })

    # print results table
    print(f"{'Weight (kg)':<12} {'Success':<10} {'Time (s)':<12} {'Max Force (N)':<15} {'Overshoot (m)':<15}")
    print("-" * 70)

    for r in results_table:
        success_str = "YES" if r['success'] else "NO"
        settling = f"{r['settling_time']:.2f}" if r['settling_time'] else "N/A"
        print(f"{r['weight']:<12} {success_str:<10} {settling:<12} {r['max_force']:<15.1f} {r['overshoot']:<15.4f}")


def analyze_telemetry(result):
    """do a detailed telemetry analysis"""
    print("\n" + "=" * 60)
    print("TELEMETRY ANALYSIS")
    print("=" * 60)

    if len(result.time) == 0:
        print("No telemetry data available")
        return

    print(f"\nData points collected: {len(result.time)}")
    print(f"Simulation duration: {result.time[-1]:.2f} s")

    # position analysis
    print(f"\nPosition:")
    print(f"  Initial: {result.position[0]:.4f} m")
    print(f"  Final: {result.position[-1]:.4f} m")
    print(f"  Max: {max(result.position):.4f} m")

    # velocity analysis
    print(f"\nVelocity:")
    print(f"  Max: {max(abs(v) for v in result.velocity):.4f} m/s")
    print(f"  Final: {result.velocity[-1]:.6f} m/s")

    # force analysis
    print(f"\nForce:")
    print(f"  Max: {max(result.force):.2f} N")
    print(f"  Average: {sum(result.force)/len(result.force):.2f} N")

    # PWM analysis
    print(f"\nPWM Control:")
    print(f"  Max: {max(result.pwm):.4f}")
    print(f"  Final: {result.pwm[-1]:.4f}")


def main():
    """run all example scenarios"""
    print("\nLIFT ACTUATOR SIMULATION - EXAMPLE SCENARIOS")
    print("=" * 60)

    # run scenarios
    result1 = run_basic_lift()
    result2 = run_heavy_user_lift()
    result3 = run_overload_scenario()
    result4 = run_fault_injection()
    run_comparison_study()

    # detailed analysis of first scenario
    analyze_telemetry(result1)

    print("\n" + "=" * 60)
    print("All scenarios completed!")
    print("=" * 60)


if __name__ == '__main__':
    main()
