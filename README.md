# lift_actuator_sim

Physics simulation for a linear actuator sling lift. Used by the Ableware voice control system.

## Classes

**`LinearActuator`** (`actuator.py`) — models the actuator mechanics (position, velocity, acceleration, PWM, stall detection).

**`UserLoad`** (`load.py`) — models the load on the actuator (user weight, required force).

**`LiftController`** (`controller.py`) — PID controller that drives the actuator toward a target position. Handles emergency stop.

## Simulation stub (Ableware integration)

`simulation_stub.py` wraps these classes in a FastAPI server so the Ableware hub can send commands and read state over HTTP.

Start it with:
```bash
python3 -m uvicorn simulation_stub:app --port 8001
```

Endpoints:
- `POST /command` — `{"command": "UP" | "DOWN" | "START" | "STOP" | "LEFT" | "RIGHT"}`
- `GET /state` — returns current actuator + controller state
- `GET /health`

The stub runs a continuous physics loop at 10ms intervals. Commands update the target position and the PID controller moves the actuator smoothly between steps.

## Running standalone

```bash
pip3 install fastapi "uvicorn[standard]" numpy
python3 -m uvicorn simulation_stub:app --port 8001
```

Test it manually:
```bash
curl -X POST http://localhost:8001/command -H "Content-Type: application/json" -d '{"command": "UP"}'
curl http://localhost:8001/state
```

## Key constants

| Constant | Value | Notes |
|---|---|---|
| `ACT_STROKE` | 0.0254 m | Total travel (1 inch) |
| `STEP_SIZE` | 0.0025 m | Distance per UP/DOWN command |
| `LOAD_MASS` | 80 kg | Approximate user weight |
| `PHYSICS_DT` | 0.01 s | Physics loop timestep |
