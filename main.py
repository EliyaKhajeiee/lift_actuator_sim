from wheelchair_sim_3d import WheelchairLiftSim3D
from core.sim_constants import ACT_STROKE
import sys
from UI.Terminal_UI import prompt_mode, show_mode_screen

def main() -> None:
    arg_mode = sys.argv[1].lower() if len(sys.argv) > 1 else None
    selected_from_ui = arg_mode is None

    while True:
        mode = arg_mode if arg_mode is not None else prompt_mode(default="demo")

        if selected_from_ui:
            show_mode_screen(mode)

        if mode == "demo":
            try:
                sim = WheelchairLiftSim3D()
                sim.run_demo()
            except KeyboardInterrupt:
                pass
            if arg_mode is not None:
                break
            continue

        if mode == "interactive":
            try:
                sim = WheelchairLiftSim3D()
                sim.run()
            except KeyboardInterrupt:
                pass
            if arg_mode is not None:
                break
            continue

        if mode == "serial":
            from serial_bridge import SerialSimulationBridge

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
            if arg_mode is not None:
                break
            continue

        if mode == "quit":
            break

        print("Usage: python main.py [demo|interactive|serial|quit]")
        if arg_mode is not None:
            break


if __name__ == "__main__":
    main()
