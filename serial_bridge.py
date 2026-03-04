import json
import threading
import time

import pybullet as p


class SerialSimulationBridge:
    """Bridge JSON-over-serial commands to a lift simulation."""

    def __init__(
        self,
        sim_factory,
        stroke_length: float,
        serial_port: str = "COM3",
        baudrate: int = 115200,
        dt: float = 1.0 / 240.0,
        headless: bool = False,
        read_timeout: float = 0.05,
    ):
        self.sim = sim_factory(headless=headless)
        self.stroke_length = stroke_length
        self.dt = dt
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.read_timeout = read_timeout

        self.ser = None
        self.running = False
        self.reader_thread = None

        self.latest_commands = {
            "target_height": None,
            "user_mass": None,
            "emergency": False,
        }
        self.need_state_reply = False
        self.rx_buf = ""
        self.lock = threading.Lock()

    def _open_serial(self) -> None:
        import serial

        self.ser = serial.Serial(
            port=self.serial_port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.read_timeout,
        )

    def start(self) -> None:
        self._open_serial()
        self.running = True
        self.reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.reader_thread.start()
        self._main_loop()

    def stop(self) -> None:
        self.running = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _serial_reader(self) -> None:
        while self.running:
            try:
                if self.ser is None:
                    time.sleep(0.05)
                    continue

                data = self.ser.read(1024)
                if not data:
                    continue

                self.rx_buf += data.decode(errors="ignore")
                while "\n" in self.rx_buf:
                    line, self.rx_buf = self.rx_buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    try:
                        msg = json.loads(line)
                    except json.JSONDecodeError:
                        print("JSON decode error:", line)
                        continue

                    self._handle_message(msg)
            except Exception as exc:
                print("serial read error:", exc)
                time.sleep(0.1)

    def _handle_message(self, msg: dict) -> None:
        msg_type = msg.get("type")

        with self.lock:
            if msg_type == "set_target":
                try:
                    self.latest_commands["target_height"] = float(msg["value"])
                except (KeyError, TypeError, ValueError):
                    print("invalid set_target payload:", msg)

            elif msg_type == "set_mass":
                try:
                    self.latest_commands["user_mass"] = float(msg["value"])
                except (KeyError, TypeError, ValueError):
                    print("invalid set_mass payload:", msg)

            elif msg_type == "emergency_stop":
                self.latest_commands["emergency"] = True

            elif msg_type == "get_state":
                self.need_state_reply = True

    def _apply_commands(self) -> None:
        with self.lock:
            target_height = self.latest_commands["target_height"]
            user_mass = self.latest_commands["user_mass"]
            emergency = self.latest_commands["emergency"]
            self.latest_commands["target_height"] = None
            self.latest_commands["user_mass"] = None
            self.latest_commands["emergency"] = False

        if target_height is not None:
            clamped = max(0.0, min(target_height, self.stroke_length))
            self.sim.ctrl.set_target_position(clamped)

        if user_mass is not None:
            self.sim.load.set_mass(max(0.0, user_mass) / 2.0)

        if emergency:
            self.sim.emergency_stop()

    def _step_simulation(self) -> None:
        required_force = self.sim.load.get_required_force(self.sim.act_L.acceleration)
        pwm = self.sim.ctrl.update(self.dt)
        self.sim.act_L.set_pwm(pwm)
        self.sim.act_R.set_pwm(pwm)
        self.sim.act_L.step(self.dt, required_force)
        self.sim.act_R.step(self.dt, required_force)
        if p.isConnected():
            p.stepSimulation()

    def _build_state(self) -> dict:
        left = self.sim.act_L.get_state()
        right = self.sim.act_R.get_state()
        ext = (left["position"] + right["position"]) / 2.0
        target = self.sim.ctrl.target_position

        return {
            "target_height": target,
            "position": ext,
            "progress": ext / max(self.stroke_length, 1e-9),
            "at_target": self.sim.ctrl.at_target(),
            "overload": self.sim.ctrl.overload_detected,
            "emergency": self.sim.ctrl.emergency_stop_triggered,
            "stalled": left["stalled"] or right["stalled"],
            "left_actuator": left,
            "right_actuator": right,
        }

    def _send_state(self) -> None:
        if self.ser is None:
            return

        payload = {"type": "state", "data": self._build_state()}
        try:
            self.ser.write((json.dumps(payload) + "\n").encode())
        except Exception as exc:
            print("serial write error:", exc)

    def _main_loop(self) -> None:
        while self.running:
            self._apply_commands()
            self._step_simulation()

            with self.lock:
                should_reply = self.need_state_reply
                self.need_state_reply = False

            if should_reply:
                self._send_state()

            time.sleep(self.dt)

