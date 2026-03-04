from serial_bridge import SerialSimulationBridge
from wheelchair_sim_3d import ACT_STROKE, WheelchairLiftSim3D


def main() -> None:
    bridge = SerialSimulationBridge(
        sim_factory=WheelchairLiftSim3D,
        stroke_length=ACT_STROKE,
        serial_port="COM3",
        baudrate=115200,
        dt=1.0 / 240.0,
        headless=False,
    )
    try:
        bridge.start()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()


if __name__ == "__main__":
    main()
