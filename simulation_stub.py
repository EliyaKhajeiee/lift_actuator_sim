"""
Ableware Simulation Stub — Drop-in for lift_actuator_sim/

HOW TO USE
----------
1. Copy this file into your lift_actuator_sim/ directory (next to actuator.py).
2. Install FastAPI + uvicorn if not already present:
       pip install fastapi uvicorn[standard]
3. Start the stub (from inside lift_actuator_sim/):
       uvicorn simulation_stub:app --port 8001
4. The Ableware Command Hub will connect automatically.

This file imports the existing simulation classes WITHOUT modifying them.
No changes to actuator.py, load.py, or controller.py are required.

API
---
POST /command   {"command": "START|STOP|UP|DOWN|LEFT|RIGHT"}
GET  /state     → current actuator + controller state
GET  /health    → {"status": "ok"}
"""

from __future__ import annotations

import asyncio
import logging
import sys
import os
from typing import Optional

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

# ---------------------------------------------------------------------------
# Import simulation classes — zero changes to the existing repo
# ---------------------------------------------------------------------------
# When run from inside lift_actuator_sim/ these imports resolve directly.
# If the repo is on sys.path already (e.g. installed as a package), they
# also work via the package __init__.py.
try:
    from actuator import LinearActuator
    from load import UserLoad
    from controller import LiftController
except ImportError:
    # Fallback: try package-style import
    from lift_actuator_sim.actuator import LinearActuator
    from lift_actuator_sim.load import UserLoad
    from lift_actuator_sim.controller import LiftController

# ---------------------------------------------------------------------------
# Constants — matching wheelchair_sim_3d.py
# ---------------------------------------------------------------------------

ACT_MAX_FORCE = 500.0      # Newtons
ACT_STROKE    = 0.0254     # 1 inch in metres
ACT_MAX_VEL   = 0.1        # m/s
ACT_MAX_ACCEL = 0.5        # m/s²
ACT_RAMP_TIME = 0.2        # seconds

LOAD_MASS     = 80.0       # kg (approximate user + seat weight)

PID_KP        = 2.0
PID_KI        = 0.1
PID_KD        = 0.5
POS_TOLERANCE = 0.001      # metres — tighter than default for smooth voice control

STEP_SIZE     = 0.0025     # 2.5 mm per UP/DOWN command
PHYSICS_DT    = 0.01       # 10 ms physics loop

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("ableware.stub")

# ---------------------------------------------------------------------------
# Simulation instances (module-level singletons)
# ---------------------------------------------------------------------------

_actuator   = LinearActuator(
    max_force=ACT_MAX_FORCE,
    stroke_length=ACT_STROKE,
    max_velocity=ACT_MAX_VEL,
    max_acceleration=ACT_MAX_ACCEL,
    acceleration_ramp_time=ACT_RAMP_TIME,
)
_load       = UserLoad(mass=LOAD_MASS)
_controller = LiftController(
    actuator=_actuator,
    load=_load,
    kp=PID_KP,
    ki=PID_KI,
    kd=PID_KD,
    position_tolerance=POS_TOLERANCE,
)

_target_position: float = 0.0   # local copy so we can clamp before calling controller
_physics_task: Optional[asyncio.Task] = None

# ---------------------------------------------------------------------------
# FastAPI app
# ---------------------------------------------------------------------------

app = FastAPI(title="Ableware Simulation Stub", version="1.0.0")


# ---------------------------------------------------------------------------
# Continuous physics loop
# ---------------------------------------------------------------------------

async def _physics_loop() -> None:
    """
    Runs the actuator + controller physics at PHYSICS_DT steps.

    This loop runs continuously in the background. Voice commands simply
    update the controller's target; the actuator moves smoothly between steps.
    """
    logger.info("Physics loop started (dt=%.3fs)", PHYSICS_DT)
    while True:
        required_force = _load.get_required_force(_actuator.acceleration)
        pwm = _controller.update(PHYSICS_DT)
        _actuator.set_pwm(pwm)
        _actuator.step(PHYSICS_DT, required_force)
        await asyncio.sleep(PHYSICS_DT)


@app.on_event("startup")
async def _startup() -> None:
    global _physics_task
    _physics_task = asyncio.create_task(_physics_loop())
    logger.info("Simulation stub ready on :8001")


@app.on_event("shutdown")
async def _shutdown() -> None:
    if _physics_task:
        _physics_task.cancel()


# ---------------------------------------------------------------------------
# Request / Response models
# ---------------------------------------------------------------------------

class CommandRequest(BaseModel):
    command: str


class StateResponse(BaseModel):
    # Actuator fields
    position: float
    velocity: float
    acceleration: float
    pwm: float
    emergency_stopped: bool
    stalled: bool
    # Controller fields
    target_position: float
    at_target: bool
    is_stable: bool


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------

@app.post("/command")
async def post_command(req: CommandRequest) -> dict:
    """
    Accept a voice/manual command and update the simulation accordingly.

    START  → clear emergency stop, resume physics
    STOP   → emergency stop
    UP     → increment target by STEP_SIZE (clamped to stroke)
    DOWN   → decrement target by STEP_SIZE (clamped to 0)
    LEFT   → placeholder (tilt axis — future)
    RIGHT  → placeholder (tilt axis — future)
    """
    global _target_position

    cmd = req.command.upper()
    logger.info("Command received: %s", cmd)

    if cmd == "START":
        _controller.reset_emergency_stop()
        logger.info("Emergency stop cleared. Physics resuming.")

    elif cmd == "STOP":
        _controller.emergency_stop()
        logger.info("Emergency stop triggered.")

    elif cmd == "UP":
        _target_position = min(_target_position + STEP_SIZE, ACT_STROKE)
        accepted = _controller.set_target_position(_target_position)
        if not accepted:
            raise HTTPException(
                status_code=400,
                detail=f"Target position {_target_position:.4f} rejected by controller.",
            )
        logger.info("Target → %.4f m (UP)", _target_position)

    elif cmd == "DOWN":
        _target_position = max(_target_position - STEP_SIZE, 0.0)
        accepted = _controller.set_target_position(_target_position)
        if not accepted:
            raise HTTPException(
                status_code=400,
                detail=f"Target position {_target_position:.4f} rejected by controller.",
            )
        logger.info("Target → %.4f m (DOWN)", _target_position)

    elif cmd in {"LEFT", "RIGHT"}:
        # Tilt axis — not yet implemented in the actuator model
        logger.info("Command %s received — tilt axis not yet implemented, ignoring.", cmd)

    else:
        raise HTTPException(status_code=422, detail=f"Unknown command: {cmd!r}")

    return {"status": "ok", "command": cmd}


@app.get("/state", response_model=StateResponse)
async def get_state() -> StateResponse:
    """Return current actuator + controller state."""
    act = _actuator.get_state()
    ctrl = _controller.get_control_state()
    return StateResponse(
        position=act["position"],
        velocity=act["velocity"],
        acceleration=act["acceleration"],
        pwm=act["pwm"],
        emergency_stopped=act["emergency_stopped"],
        stalled=act["stalled"],
        target_position=ctrl["target_position"],
        at_target=ctrl["at_target"],
        is_stable=ctrl["is_stable"],
    )


@app.get("/health")
async def health() -> dict:
    return {"status": "ok"}
