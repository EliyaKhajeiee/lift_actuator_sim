#!/usr/bin/env python3
"""
3D Wheelchair Sling Lift Simulation — PyBullet (Demo Edition)

Simulates the assistive sling-lift device:
  - Manual wheelchair with tubular steel frame and spoked wheels
  - Rigid sling board sitting on the seat cushion
  - Two linear actuators under the sling that push it up ~1 inch
  - 1-inch lift creates clearance to pull pants through the gap

Physics driven by LinearActuator / LiftController / UserLoad modules.
PyBullet handles rendering and the interactive GUI sliders.

Usage:
    python3 wheelchair_sim_3d.py

Sliders (left panel):
    User Weight (kg)            — 40–220 kg; affects actuator load
    Lift Target (0→1)           — 0 = down, 1 = full 1-inch lift
    Max Force Per Actuator (N)  — lower this to see stall
    Speed Multiplier            — 0.1× slow-motion to 4× fast
"""

import sys
import os
import time
import math

import numpy as np


from keyInput import KeyReader

from UI import build_demo_scenarios_panel, create_demo_live, update_demo_live
from UI.interactive_screen_ui import (
    show_interactive_screen as show_interactive_live_screen,
    create_interactive_live,
    update_interactive_live,
)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pybullet as p
import pybullet_data

from actuator import LinearActuator
from load import UserLoad
from controller import LiftController

from core.sim_constants import *
from core.sim_pybHelper import _q, _box, _cyl


# Visual-only (no collision) — for kinematic animated bodies

def _vbox(half, pos, orn=None, color=None):
    orn = orn or _q()
    vis = (p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
           if color else -1)
    return p.createMultiBody(0.001, -1, vis, pos, orn)


def _vcyl(r, h, pos, orn=None, color=None):
    orn = orn or _q()
    vis = (p.createVisualShape(p.GEOM_CYLINDER, radius=r, length=h, rgbaColor=color)
           if color else -1)
    return p.createMultiBody(0.001, -1, vis, pos, orn)


def _vsph(r, pos, color=None):
    vis = (p.createVisualShape(p.GEOM_SPHERE, radius=r, rgbaColor=color)
           if color else -1)
    return p.createMultiBody(0.001, -1, vis, pos)


def _tube(x0, y0, z0, x1, y1, z1, r=None, color=None):
    """Draw a cylinder tube from point A to point B."""
    r = r if r is not None else TUBE_R
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = math.sqrt(dx*dx + dy*dy + dz*dz)
    if length < 1e-6:
        return
    cx, cy, cz = (x0+x1)/2, (y0+y1)/2, (z0+z1)/2
    tx, ty, tz = dx/length, dy/length, dz/length
    cos_a = tz  # dot(Z_axis, target)
    if abs(cos_a - 1.0) < 1e-6:
        orn = _q()
    elif abs(cos_a + 1.0) < 1e-6:
        orn = (1.0, 0.0, 0.0, 0.0)
    else:
        ax, ay = -ty, tx
        n = math.sqrt(ax*ax + ay*ay)
        ax, ay = ax/n, ay/n
        a = math.acos(max(-1.0, min(1.0, cos_a)))
        s = math.sin(a / 2)
        orn = (ax*s, ay*s, 0.0, math.cos(a / 2))
    _cyl(r, length, [cx, cy, cz], orn, color)


def _move(body_id, pos, orn=None):
    p.resetBasePositionAndOrientation(body_id, pos, orn or _q())


def _recolor(body_id, rgba):
    p.changeVisualShape(body_id, -1, rgbaColor=rgba)


WHEEL_ORN = _q(math.pi / 2, 0, 0)   # lay cylinder on its side


# ── Main simulation class ──────────────────────────────────────────────────

class WheelchairLiftSim3D:
    """
    Interactive 3D wheelchair sling-lift simulation.

    LinearActuator + LiftController + UserLoad handle all physics.
    PyBullet provides rendering, real-time GUI, and visual feedback.
    """

    def __init__(self, headless: bool = False):
        self._headless = headless
        self._connect()

        p.setGravity(0, 0, 0)   # physics modules own gravity
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        if not self._headless:
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
            p.resetDebugVisualizerCamera(
                cameraDistance=1.95,
                cameraYaw=36,
                cameraPitch=-22,
                cameraTargetPosition=[-0.02, 0.0, 0.66])

        self._init_physics(user_mass=80.0, max_force=750.0)

        self._build_floor()
        self._build_wheelchair()
        self._build_sling()
        self._build_actuators()

        if not self._headless:
            self._create_sliders()
            self._build_labels()
            self._build_lift_indicator()

        self._txt         = {}   # HUD text item IDs
        self._arrows      = {}   # force arrow debug-line IDs
        self._ind_line_id = -1   # animated lift-indicator line
        self._duty_on     = 0    # frames spent actively driving
        self._duty_total  = 0    # total frames (for duty-cycle %)


    # ── Connection ────────────────────────────────────────────────────────

    def _connect(self):
        if self._headless:
            self.client = p.connect(p.DIRECT)
            return
        try:
            self.client = p.connect(p.GUI)
        except Exception as exc:
            print(f"[ERROR] Cannot open PyBullet GUI: {exc}")
            print("Make sure a display is available (X11 / VNC / WSL).")
            sys.exit(1)

    # ── Physics init ──────────────────────────────────────────────────────

    def _init_physics(self, user_mass: float, max_force: float):
        self.user_mass = user_mass
        self.max_force = max_force

        self.act_L = LinearActuator(
            max_force=max_force, stroke_length=ACT_STROKE,
            max_velocity=ACT_MAX_VEL, max_acceleration=ACT_MAX_ACC,
            acceleration_ramp_time=ACT_RAMP_T)
        self.act_R = LinearActuator(
            max_force=max_force, stroke_length=ACT_STROKE,
            max_velocity=ACT_MAX_VEL, max_acceleration=ACT_MAX_ACC,
            acceleration_ramp_time=ACT_RAMP_T)
        self.load = UserLoad(mass=user_mass / 2.0)
        self.ctrl = LiftController(
            actuator=self.act_L, load=self.load,
            kp=10.0, ki=0.12, kd=2.5,
            position_tolerance=0.002)   # 2 mm — realistic real-world tolerance
        # Lower ki reduces integrator windup on the long 4-inch approach.
        # Higher kd damps overshoot when decelerating near target.
        self.ctrl.set_target_position(0.0)

    # ── Safety ────────────────────────────────────────────────────────────

    def emergency_stop(self):
        """Halt both actuators and the controller."""
        self.ctrl.emergency_stop_triggered = True
        self.act_L.emergency_stop()
        self.act_R.emergency_stop()

    # ── Floor ─────────────────────────────────────────────────────────────

    def _build_floor(self):
        # Near-black charcoal base — polished-showroom feel
        _box([4.5, 4.5, 0.012], [0, 0, -0.012], color=C_FLOOR_A)
        # Very fine grid at 50 cm intervals (barely visible — professional)
        for i in range(-8, 9):
            _box([4.5, 0.0010, 0.0002], [0, i * 0.50, 0.0005], color=C_FLOOR_B)
            _box([0.0010, 4.5, 0.0002], [i * 0.50, 0, 0.0005], color=C_FLOOR_B)
        # Brighter centre reference cross
        C_GRID_CTR = [0.26, 0.28, 0.33, 1.0]
        _box([4.5, 0.0022, 0.0003], [0, 0, 0.0006], color=C_GRID_CTR)
        _box([0.0022, 4.5, 0.0003], [0, 0, 0.0006], color=C_GRID_CTR)

    # ── Wheelchair frame ──────────────────────────────────────────────────

    def _build_wheelchair(self):
        sz  = SEAT_H
        bx  = -SEAT_D / 2        # rear edge X
        fx  =  SEAT_D / 2        # front edge X

        # ── seat cushion ──────────────────────────────────────────────
        _box([SEAT_D/2 - 0.012, SEAT_W/2 - 0.012, CUSHION_T/2],
             [0, 0, sz - CUSHION_T/2], color=C_CUSHION)
        # side bolsters
        for y in (SEAT_W/2 - 0.022, -(SEAT_W/2 - 0.022)):
            _box([SEAT_D/2 - 0.015, 0.020, CUSHION_T/2 + 0.006],
                 [0, y, sz - CUSHION_T/2], color=C_CUSHION)

        # ── seat frame rails ──────────────────────────────────────────
        for y in (SEAT_W/2 - 0.022, -(SEAT_W/2 - 0.022)):
            _tube(bx + 0.02, y, sz - CUSHION_T - 0.008,
                   fx - 0.02, y, sz - CUSHION_T - 0.008, color=C_FRAME_MD)
        _tube(-0.06, -(SEAT_W/2-0.022), sz-CUSHION_T-0.008,
               -0.06,  (SEAT_W/2-0.022), sz-CUSHION_T-0.008, color=C_FRAME_MD)
        _tube( 0.06, -(SEAT_W/2-0.022), sz-CUSHION_T-0.008,
                0.06,  (SEAT_W/2-0.022), sz-CUSHION_T-0.008, color=C_FRAME_MD)

        # ── main side frames ──────────────────────────────────────────
        for y in (SEAT_W/2, -SEAT_W/2):
            # rear vertical post
            _tube(-0.10, y, 0.04, -0.10, y, sz - CUSHION_T - 0.015,
                  r=TUBE_R*1.2, color=C_FRAME_DK)
            # front vertical post
            _tube(fx - 0.03, y, 0.04, fx - 0.03, y, sz - CUSHION_T - 0.015,
                  r=TUBE_R*1.1, color=C_FRAME_DK)
            # bottom rail
            _tube(-0.10, y, 0.040, fx - 0.03, y, 0.040,
                  r=TUBE_R, color=C_FRAME_DK)
            # diagonal brace
            _tube(-0.10, y, 0.040, fx - 0.03, y, sz - CUSHION_T - 0.015,
                  r=TUBE_R*0.85, color=C_FRAME_MD)

        # cross braces under seat
        _tube(-0.10, -SEAT_W/2, 0.040, -0.10, SEAT_W/2, 0.040, color=C_FRAME_DK)
        _tube(fx-0.03, -SEAT_W/2, 0.040, fx-0.03, SEAT_W/2, 0.040, color=C_FRAME_DK)

        # ── backrest ──────────────────────────────────────────────────
        for y in (SEAT_W/2 - 0.030, -(SEAT_W/2 - 0.030)):
            _tube(bx+0.015, y, sz, bx+0.015, y, sz + BACK_H, color=C_FRAME_MD)
        _tube(bx+0.015, -(SEAT_W/2-0.030), sz + BACK_H,
               bx+0.015,  (SEAT_W/2-0.030), sz + BACK_H, color=C_FRAME_MD)
        _tube(bx+0.015, -(SEAT_W/2-0.030), sz + BACK_H*0.55,
               bx+0.015,  (SEAT_W/2-0.030), sz + BACK_H*0.55, color=C_FRAME_MD)
        # upholstery panel
        _box([0.018, SEAT_W/2 - 0.055, BACK_H/2 - 0.020],
             [bx + 0.022, 0, sz + BACK_H/2], color=C_CUSHION)

        # ── push handles ──────────────────────────────────────────────
        handle_z = sz + BACK_H + 0.11
        for y in (SEAT_W/2 - 0.025, -(SEAT_W/2 - 0.025)):
            _tube(bx+0.015, y, sz + BACK_H,
                   bx - 0.015, y, handle_z, r=TUBE_R, color=C_FRAME_MD)
        _tube(bx-0.015, -(SEAT_W/2-0.025), handle_z,
               bx-0.015,  (SEAT_W/2-0.025), handle_z,
               r=TUBE_R*1.1, color=C_FRAME_LT)

        # ── rear wheels ───────────────────────────────────────────────
        wx = -0.08
        for y in (SEAT_W/2 + 0.048, -(SEAT_W/2 + 0.048)):
            _cyl(WHEEL_R,          WHEEL_T,        [wx, y, WHEEL_R], WHEEL_ORN, C_TYRE)
            _cyl(WHEEL_R - 0.006,  WHEEL_T * 0.45, [wx, y, WHEEL_R], WHEEL_ORN, C_RIM)
            _cyl(WHEEL_R * 0.82,   WHEEL_T * 0.28, [wx, y, WHEEL_R], WHEEL_ORN, C_SPOKE)
            _cyl(0.028,            WHEEL_T * 0.80, [wx, y, WHEEL_R], WHEEL_ORN, C_HUB)
            # 6 spokes as full-diameter cylinders crossing the hub
            for angle_deg in range(0, 180, 30):
                th = math.radians(angle_deg)
                _cyl(0.004, WHEEL_R * 1.85,
                     [wx, y, WHEEL_R], _q(0.0, th, 0.0), C_SPOKE)
            # axle stub
            _cyl(0.018, 0.055, [wx, y, WHEEL_R], WHEEL_ORN, C_HUB)

        # ── front casters ─────────────────────────────────────────────
        cx = fx + 0.036
        for y in (SEAT_W/2 - 0.072, -(SEAT_W/2 - 0.072)):
            _cyl(CASTER_R,        CASTER_T,        [cx, y, CASTER_R], WHEEL_ORN, C_TYRE)
            _cyl(CASTER_R * 0.68, CASTER_T * 0.55, [cx, y, CASTER_R], WHEEL_ORN, C_RIM)
            _cyl(0.016,           CASTER_T * 0.75, [cx, y, CASTER_R], WHEEL_ORN, C_HUB)
            _tube(cx, y, CASTER_R * 2.0,
                   cx, y, CASTER_R * 2.0 + 0.072, r=TUBE_R*0.8, color=C_FRAME_MD)
            _tube(cx, y, CASTER_R*2.0 + 0.072,
                   fx - 0.03, y, sz - CUSHION_T - 0.018,
                   r=TUBE_R*0.8, color=C_FRAME_MD)

        # ── armrests ──────────────────────────────────────────────────
        arm_z = sz + 0.240
        for y in (SEAT_W/2 + 0.020, -(SEAT_W/2 + 0.020)):
            _box([0.185, 0.022, 0.014], [0.005, y, arm_z], color=C_FRAME_LT)
            _tube(0.005, y, sz - 0.010, 0.005, y, arm_z - 0.014,
                  r=TUBE_R*0.85, color=C_FRAME_MD)

        # ── footrests ─────────────────────────────────────────────────
        for y in (SEAT_W/2 - 0.095, -(SEAT_W/2 - 0.095)):
            _tube(fx - 0.025, y, sz - CUSHION_T - 0.018,
                   fx + 0.12, y, 0.15,
                   r=TUBE_R, color=C_FRAME_MD)
        _tube(fx + 0.12, -(SEAT_W/2-0.095), 0.150,
               fx + 0.12,  (SEAT_W/2-0.095), 0.150,
               r=TUBE_R, color=C_FRAME_MD)
        _box([0.070, SEAT_W/2 - 0.110, 0.008],
             [fx + 0.12, 0, 0.143], color=C_FRAME_LT)

    # ── Side actuators + sling connection ────────────────────────────────

    def _build_actuators(self):
        """
        Two vertical linear actuators, one on each side of the seat.

        Modelled on a PA-14 class actuator (1000 N, 4-inch stroke, 12/24 V DC).

        Each side:
          housing  — fixed outer tube (static)
          shaft    — inner rod, animated upward as actuator extends
          end-cap  — clevis / mounting point at bottom
          arm      — horizontal yoke from shaft top, extends inward to sling
          straps   — two nylon webbing straps (front + rear) arm → sling corners
        """
        body_base_z        = SEAT_H
        body_ctr_z         = body_base_z + ACT_BODY_H / 2
        shaft_base_z       = body_base_z + ACT_BODY_H
        self._shaft_base_z = shaft_base_z

        # At rest (ext=0) the arm sits at shaft_base + shaft_h
        arm_z0            = shaft_base_z + ACT_SHAFT_H
        sling_top_z0      = SEAT_H + SLING_T
        self._strap_len   = arm_z0 - sling_top_z0   # constant strap length

        self._housing_ids = []
        self._shaft_ids   = []
        self._arm_ids     = []
        self._strap_ids   = []   # 4 straps: L-front, L-rear, R-front, R-rear

        # Motor housing height fraction (like a real PA-14: fatter bottom section)
        motor_h = ACT_BODY_H * 0.38   # gearbox / motor end
        tube_h  = ACT_BODY_H * 0.62   # actuator barrel

        for sign, y in ((+1, ACT_SIDE_Y), (-1, -ACT_SIDE_Y)):

            # ── Motor / gearbox body (darker, wider — bottom) ──────────
            motor_cz = body_base_z + motor_h / 2
            _cyl(ACT_R * 1.28, motor_h,
                 [ACT_X, y, motor_cz], color=C_ACT_MOTOR)

            # Mounting flange ring between motor and barrel
            _cyl(ACT_R * 1.42, 0.012,
                 [ACT_X, y, body_base_z + motor_h], color=C_CAP)

            # Bottom clevis ear-plate + clevis pin
            _box([0.022, 0.018, 0.030],
                 [ACT_X, y, body_base_z - 0.030], color=C_CAP)
            _cyl(0.006, 0.042, [ACT_X, y, body_base_z - 0.032],
                 _q(math.pi/2, 0, 0), C_FRAME_LT)

            # Side mount bracket (attaches to wheelchair frame)
            bkt_y = y - sign * (ACT_R * 1.28 + 0.014)
            _box([0.042, 0.013, 0.056],
                 [ACT_X, bkt_y, motor_cz], color=C_FRAME_DK)
            # Bracket-to-frame bolt heads (two small cylinders)
            for dz in (-0.014, +0.014):
                _cyl(0.005, 0.005,
                     [ACT_X, bkt_y - sign * 0.013, motor_cz + dz],
                     _q(math.pi/2, 0, 0), C_FRAME_LT)

            # Wiring / power cable exiting motor body
            cable_y = y + sign * ACT_R * 0.88
            _cyl(0.007, 0.058,
                 [ACT_X - 0.032, cable_y, body_base_z + 0.032],
                 _q(0, math.pi * 0.38, 0), C_CABLE)

            # ── Actuator barrel / tube (brushed aluminum, upper) ───────
            tube_cz = body_base_z + motor_h + tube_h / 2
            h = _cyl(ACT_R, tube_h,
                     [ACT_X, y, tube_cz], color=C_ACT_HOUSE)
            self._housing_ids.append(h)

            # Top end-cap disc
            _cyl(ACT_R * 1.14, ACT_CAP_H,
                 [ACT_X, y, body_base_z + ACT_BODY_H + ACT_CAP_H / 2],
                 color=C_CAP)

            # ── Inner rod / shaft (animated) ───────────────────────────
            shaft_ctr_z = shaft_base_z + ACT_SHAFT_H / 2
            s = _vcyl(ACT_R * 0.46, ACT_SHAFT_H,
                      [ACT_X, y, shaft_ctr_z], color=C_ACT_ROD)
            self._shaft_ids.append(s)

            # ── Horizontal yoke arm (animated, chrome) ─────────────────
            arm_len = ACT_SIDE_Y - ARM_INNER_Y
            arm_cy  = (y + sign * ARM_INNER_Y) / 2
            a = _vcyl(ARM_R, arm_len,
                      [ACT_X, arm_cy, arm_z0], _q(math.pi/2, 0, 0), C_ARM)
            self._arm_ids.append(a)
            # Ball joint at arm inner tip
            _vsph(ARM_R * 1.8, [ACT_X, sign * ARM_INNER_Y, arm_z0], C_ARM)

            # ── Nylon webbing suspension straps (animated) ─────────────
            strap_ctr_z = arm_z0 - self._strap_len / 2
            for tx in (TIE_X_F, TIE_X_R):
                t = _vbox([TIE_W / 2, SLING_STRAP_W / 2, self._strap_len / 2],
                          [tx, sign * ARM_INNER_Y, strap_ctr_z],
                          color=C_TIE)
                self._strap_ids.append(t)

        # ── Structural cross-braces (span motor-body width) ────────────
        xb_r = ACT_R * 1.28   # match motor housing radius
        _tube(ACT_X, -ACT_SIDE_Y + xb_r, ACT_XBRACE_Z,
              ACT_X,  ACT_SIDE_Y - xb_r, ACT_XBRACE_Z,
              r=TUBE_R * 1.5, color=C_FRAME_DK)
        _tube(ACT_X, -ACT_SIDE_Y + xb_r, ACT_XBRACE_Z + ACT_BODY_H * 0.38,
              ACT_X,  ACT_SIDE_Y - xb_r, ACT_XBRACE_Z + ACT_BODY_H * 0.38,
              r=TUBE_R * 1.2, color=C_FRAME_MD)

    # ── Sling ─────────────────────────────────────────────────────────────

    def _build_sling(self):
        """
        Sling assembly — matches a real lift sling:
          • 3 transverse nylon straps (front, mid, rear) spanning seat width
          • 2 longitudinal side rails connecting strap ends
          • All parts animate upward together
        """
        z0 = SEAT_H + SLING_T / 2
        self._sling_parts = []   # (body_id, x_offset, y_offset)

        # ── 3 transverse nylon straps ──────────────────────────────────
        for sx in (TIE_X_F, 0.0, TIE_X_R):
            bid = _vbox([SLING_STRAP_W / 2, SLING_W / 2, SLING_T / 2 + 0.005],
                        [sx, 0, z0], color=C_SLING_DN)
            self._sling_parts.append((bid, sx, 0.0))

        # ── 2 longitudinal side rails (span front-strap to rear-strap) ─
        rail_hw = (TIE_X_F - TIE_X_R) / 2.0   # half-length of rail
        for sy in (SLING_W / 2 - SLING_RAIL_W / 2,
                   -(SLING_W / 2 - SLING_RAIL_W / 2)):
            bid = _vbox([rail_hw, SLING_RAIL_W / 2, SLING_T / 2 + 0.003],
                        [0.0, sy, z0], color=C_SLING_EDG)
            self._sling_parts.append((bid, 0.0, sy))

        # back-compat: sling_id points to first strap
        self.sling_id     = self._sling_parts[0][0]
        self._sling_edges = []   # now handled via _sling_parts

    # ── GUI sliders ───────────────────────────────────────────────────────

    def _create_sliders(self):
        self.sl_weight = p.addUserDebugParameter(
            "User Weight (kg)  [40 – 220]", 40, 220, 80)
        self.sl_target = p.addUserDebugParameter(
            "Lift Target  ( 0 = down   →   1 = full 4-inch lift )", 0.0, 1.0, 0.0)
        self.sl_force  = p.addUserDebugParameter(
            "Actuator Rated Force per unit (N)", 200, 3000, 1000)
        self.sl_volts  = p.addUserDebugParameter(
            "Supply Voltage (V)  [12 or 24]", 12, 24, 12)
        self.sl_speed  = p.addUserDebugParameter(
            "Sim Speed  (0.25 = slow-mo   1 = real-time   4 = fast)", 0.25, 4.0, 1.0)

    # ── Static 3-D annotations ───────────────────────────────────────────

    def _build_labels(self):
        """Minimal 3-D annotations — clean enough for a live demo."""
        # Single floating title above the device (centred)
        p.addUserDebugText(
            "SLING LIFT ACTUATOR SYSTEM",
            [0.0, 0.0, SEAT_H + ACT_BODY_H + ACT_SHAFT_H + ACT_STROKE + 0.30],
            textColorRGB=[0.90, 0.92, 0.95], textSize=1.10, lifeTime=0)
        p.addUserDebugText(
            "PA-14 Class  |  4-inch Stroke  |  12 / 24 V DC",
            [0.0, 0.0, SEAT_H + ACT_BODY_H + ACT_SHAFT_H + ACT_STROKE + 0.18],
            textColorRGB=C_LABEL_Y, textSize=0.75, lifeTime=0)

    def _build_lift_indicator(self):
        """
        Static vertical ruler to the right of the right actuator,
        showing the full 4-inch stroke with percentage tick marks.
        """
        z_bot = SEAT_H + ACT_BODY_H + ACT_SHAFT_H
        z_top = z_bot + ACT_STROKE
        rx    = RULER_X

        # Ruler spine
        p.addUserDebugLine([rx, 0, z_bot], [rx, 0, z_top],
                           C_RULER, lineWidth=3, lifeTime=0)

        # Tick marks and labels at 0 %, 25 %, 50 %, 75 %, 100 %
        for frac, label in ((0.0, "0%  (down)"),
                             (0.25, "25%"),
                             (0.50, "50%"),
                             (0.75, "75%"),
                             (1.0, "100% (4 in)")):
            zt = z_bot + frac * ACT_STROKE
            p.addUserDebugLine([rx - 0.016, 0, zt], [rx + 0.016, 0, zt],
                               C_RULER, lineWidth=2, lifeTime=0)
            p.addUserDebugText(label, [rx + 0.022, 0, zt],
                               textColorRGB=C_RULER, textSize=0.62, lifeTime=0)

    # ── Per-frame visual updates ──────────────────────────────────────────

    def _update_visuals(self, ext: float, stalled: bool,
                        overloaded: bool, at_target: bool,
                        force_per_act: float):
        # Colour coding
        # Shaft colour shows operational state; housing stays aluminum
        if stalled or overloaded:
            shaft_c = C_ACT_DEAD
            sling_c = C_SLING_DN
            arr_c   = [0.95, 0.15, 0.10]
        elif at_target and ext > 0.002:
            shaft_c = C_ACT_OK
            sling_c = C_SLING_UP
            arr_c   = [0.18, 0.90, 0.30]
        elif ext > ACT_STROKE * 0.72:
            shaft_c = C_ACT_WARN
            sling_c = C_SLING_DN
            arr_c   = [0.95, 0.70, 0.10]
        elif ext > 0.002:
            shaft_c = C_ACT_OK
            sling_c = C_SLING_DN
            arr_c   = [0.18, 0.90, 0.30]
        else:
            shaft_c = C_ACT_ROD
            sling_c = C_SLING_DN
            arr_c   = [0.55, 0.55, 0.60]
        act_c = shaft_c   # kept for arm recolor

        arm_z      = self._shaft_base_z + ACT_SHAFT_H + ext
        sling_top  = SEAT_H + SLING_T + ext
        strap_ctr  = (arm_z + sling_top) / 2
        arm_orn    = _q(math.pi / 2, 0, 0)

        # Force arrow scale: 0.05 m per 100 N, max 0.30 m
        arrow_len = min(0.30, force_per_act / 100.0 * 0.05)

        strap_idx = 0
        for i, (sign, y) in enumerate(((+1, ACT_SIDE_Y), (-1, -ACT_SIDE_Y))):
            # shaft rises with extension
            shaft_ctr_z = self._shaft_base_z + ACT_SHAFT_H / 2 + ext
            _move(self._shaft_ids[i], [ACT_X, y, shaft_ctr_z])
            _recolor(self._shaft_ids[i],   shaft_c)
            _recolor(self._housing_ids[i], C_ACT_HOUSE)   # always aluminum

            # yoke arm rises too
            arm_cy = (y + sign * ARM_INNER_Y) / 2
            _move(self._arm_ids[i], [ACT_X, arm_cy, arm_z], arm_orn)
            _recolor(self._arm_ids[i], act_c)

            # webbing straps
            for tx in (TIE_X_F, TIE_X_R):
                _move(self._strap_ids[strap_idx],
                      [tx, sign * ARM_INNER_Y, strap_ctr])
                _recolor(self._strap_ids[strap_idx], C_TIE)
                strap_idx += 1

            # force arrows: green upward arrows at arm inner tips
            tip_y  = sign * ARM_INNER_Y
            base   = [ACT_X, tip_y, arm_z]
            tip    = [ACT_X, tip_y, arm_z + arrow_len]
            key    = f'arr{i}'
            if key in self._arrows:
                p.addUserDebugLine(base, tip, arr_c, lineWidth=4,
                                   replaceItemUniqueId=self._arrows[key])
            else:
                self._arrows[key] = p.addUserDebugLine(
                    base, tip, arr_c, lineWidth=4, lifeTime=0)

        # sling assembly (straps + rails)
        sz = SEAT_H + SLING_T / 2 + ext
        for bid, ox, oy in self._sling_parts:
            _move(bid, [ox, oy, sz])
            _recolor(bid, sling_c)

        # lift-position indicator dot on the ruler
        z_bot = self._shaft_base_z + ACT_SHAFT_H
        z_ind = z_bot + ext
        rx    = RULER_X
        ind_from = [rx, 0, z_ind]
        ind_to   = [rx - 0.032, 0, z_ind]
        if self._ind_line_id >= 0:
            self._ind_line_id = p.addUserDebugLine(
                ind_from, ind_to, [1.0, 0.85, 0.0], lineWidth=5,
                replaceItemUniqueId=self._ind_line_id, lifeTime=0)
        else:
            self._ind_line_id = p.addUserDebugLine(
                ind_from, ind_to, [1.0, 0.85, 0.0], lineWidth=5, lifeTime=0)

    # ── Main run loop ─────────────────────────────────────────────────────

    def run(self):
        BASE_DT = 0.01
        RENDER_EVERY = 2
        CTRL_MIN_WEIGHT = 40.0
        CTRL_MAX_WEIGHT = 220.0
        CTRL_MIN_TARGET = 0.0
        CTRL_MAX_TARGET = 1.0
        CTRL_MIN_FORCE = 200.0
        CTRL_MAX_FORCE = 3000.0
        CTRL_MIN_VOLTS = 12.0
        CTRL_MAX_VOLTS = 24.0
        CTRL_MIN_SPEED = 0.25
        CTRL_MAX_SPEED = 4.0

        prev_weight = -1.0
        prev_force = -1.0
        step = 0
        duty_window = 200
        ctrl_state = {
            "weight": 80.0,
            "target": 0.0,
            "force": 1000.0,
            "volts": 12.0,
            "speed": 1.0,
        }

        initial_ext = (self.act_L.position + self.act_R.position) / 2.0
        initial_state = {
            "user_mass": self.load.mass * 2.0,
            "ext_m": initial_ext,
            "pwm": self.act_L.pwm_input,
            "cap_force_each": self.act_L.max_force,
            "stalled": self.act_L.stalled or self.act_R.stalled,
            "overloaded": self.ctrl.overload_detected,
            "at_target": self.ctrl.at_target(),
            "supply_v": 12.0,
            "duty_pct": 0.0,
            "target_position": self.ctrl.target_position,
            "target": self.ctrl.target_position / max(ACT_STROKE, 1e-9),
            "speed": 1.0,
        }

        try:
            show_interactive_live_screen()
            with create_interactive_live(initial_state, refresh_per_second=10) as live:
                with KeyReader() as key_reader:
                    while p.isConnected():
                        t0 = time.perf_counter()
                        should_quit = False

                        for ch in key_reader.poll():
                            c = ch.lower()
                            if c == "q":
                                should_quit = True
                                break
                            elif c == "j":
                                ctrl_state["weight"] -= 1.0
                            elif c == "k":
                                ctrl_state["weight"] += 1.0
                            elif c == "n":
                                ctrl_state["target"] -= 0.02
                            elif c == "m":
                                ctrl_state["target"] += 0.02
                            elif c == "u":
                                ctrl_state["force"] -= 50.0
                            elif c == "i":
                                ctrl_state["force"] += 50.0
                            elif c == "v":
                                ctrl_state["volts"] -= 1.0
                            elif c == "b":
                                ctrl_state["volts"] += 1.0
                            elif c == "1":
                                ctrl_state["speed"] -= 0.05
                            elif c == "2":
                                ctrl_state["speed"] += 0.05

                        if should_quit:
                            break

                        ctrl_state["weight"] = max(CTRL_MIN_WEIGHT, min(CTRL_MAX_WEIGHT, ctrl_state["weight"]))
                        ctrl_state["target"] = max(CTRL_MIN_TARGET, min(CTRL_MAX_TARGET, ctrl_state["target"]))
                        ctrl_state["force"] = max(CTRL_MIN_FORCE, min(CTRL_MAX_FORCE, ctrl_state["force"]))
                        ctrl_state["volts"] = max(CTRL_MIN_VOLTS, min(CTRL_MAX_VOLTS, ctrl_state["volts"]))
                        ctrl_state["speed"] = max(CTRL_MIN_SPEED, min(CTRL_MAX_SPEED, ctrl_state["speed"]))

                        user_mass = ctrl_state["weight"]
                        target_in = ctrl_state["target"]
                        max_force = ctrl_state["force"]
                        supply_v = ctrl_state["volts"]
                        speed_mult = ctrl_state["speed"]
                        target_m = target_in * ACT_STROKE

                        if abs(user_mass - prev_weight) > 0.05:
                            self.load.set_mass(user_mass / 2.0)
                            prev_weight = user_mass

                        if abs(max_force - prev_force) > 0.5:
                            self.act_L.max_force = max_force
                            self.act_R.max_force = max_force
                            prev_force = max_force

                        if abs(target_m - self.ctrl.target_position) > 0.0001:
                            clamped = max(0.0, min(target_m, ACT_STROKE))
                            if not self.ctrl.set_target_position(clamped):
                                self.ctrl.set_target_position(0.0)

                        n_steps = max(1, round(speed_mult))
                        pwm = 0.0
                        req_force = 0.0

                        for _ in range(n_steps):
                            req_force = self.load.get_required_force(self.act_L.acceleration)
                            pwm = self.ctrl.update(BASE_DT)
                            self.act_L.set_pwm(pwm)
                            self.act_R.set_pwm(pwm)
                            self.act_L.step(BASE_DT, req_force)
                            self.act_R.step(BASE_DT, req_force)
                            p.stepSimulation()

                            self._duty_total += 1
                            if abs(pwm) > 0.02 and not self.act_L.stalled:
                                self._duty_on += 1

                        ext = (self.act_L.position + self.act_R.position) / 2.0
                        stalled = self.act_L.stalled or self.act_R.stalled
                        overloaded = self.ctrl.overload_detected
                        at_target = self.ctrl.at_target()

                        window = max(1, duty_window)
                        if self._duty_total > window * 3:
                            self._duty_on = int(self._duty_on * window / self._duty_total)
                            self._duty_total = window
                        duty_pct = self._duty_on / max(1, self._duty_total) * 100.0

                        step += 1
                        if step % RENDER_EVERY == 0:
                            self._update_visuals(ext, stalled, overloaded, at_target, force_per_act=req_force)

                        update_interactive_live(
                            live,
                            {
                                "user_mass": user_mass,
                                "ext_m": ext,
                                "pwm": pwm,
                                "cap_force_each": max_force,
                                "stalled": stalled,
                                "overloaded": overloaded,
                                "at_target": at_target,
                                "supply_v": supply_v,
                                "duty_pct": duty_pct,
                                "target_position": self.ctrl.target_position,
                                "target": ctrl_state["target"],
                                "speed": ctrl_state["speed"],
                            },
                        )

                        frame_real_dt = BASE_DT * n_steps / speed_mult
                        elapsed = time.perf_counter() - t0
                        slack = frame_real_dt - elapsed
                        if slack > 0:
                            time.sleep(slack)

        except KeyboardInterrupt:
            print("\nSimulation stopped.")
        finally:
            if p.isConnected():
                p.disconnect()


    # ── Automated demo ────────────────────────────────────────────────────

    def run_demo(self):
        """
        Self-running demo — no slider interaction needed.

        Cycles through 4 weight scenarios automatically at 5× real-time
        so every lift, hold, and lower is clearly visible.

        Scenarios:
          1. Light user   (60 kg / 132 lb)  — lifts easily
          2. Standard user (80 kg / 176 lb) — nominal design case
          3. Heavy user  (130 kg / 286 lb)  — high-force actuator needed
          4. Overload test (180 kg, low rated force) — intentional stall
        """
        SCENARIOS = [
            {"label": "LIGHT USER",      "mass_kg":  60, "force": 1000},
            {"label": "STANDARD USER",   "mass_kg":  80, "force": 1000},
            {"label": "HEAVY USER",      "mass_kg": 130, "force": 1500},
            {"label": "OVERLOAD TEST",   "mass_kg": 180, "force":  500},
        ]
        DEMO_STEPS  = 5       # physics steps per wall-clock frame → 5× real-time
        BASE_DT     = 0.01
        HOLD_TOP_S  = 3.0     # wall-clock seconds to hold at full extension
        HOLD_BTM_S  = 1.8     # wall-clock seconds between scenarios

        si          = 0
        phase       = "LIFTING"
        hold_start  = None
        demo_txt    = {}
        step        = 0

        z_top = SEAT_H + ACT_BODY_H + ACT_SHAFT_H + ACT_STROKE + 0.55
        tx    = 0.55

        # z positions for each HUD row (top → bottom)
        Zs = [z_top + 0.17,   # 0  top border
              z_top + 0.07,   # 1  title
              z_top - 0.04,   # 2  scenario counter
              z_top - 0.14,   # 3  mid border
              z_top - 0.28,   # 4  scenario label  (big)
              z_top - 0.40,   # 5  mass
              z_top - 0.52,   # 6  divider
              z_top - 0.61,   # 7  phase indicator (big)
              z_top - 0.73,   # 8  progress bar
              z_top - 0.83,   # 9  lift measurement
              z_top - 0.95,   # 10 divider
              z_top - 1.04,   # 11 FORCE CALC header
              z_top - 1.14,   # 12 total gravity
              z_top - 1.24,   # 13 per actuator
              z_top - 1.34,   # 14 rated / OK
              z_top - 1.45,   # 15 bottom border
              ]

        def _reset_and_load(idx):
            s = SCENARIOS[idx % len(SCENARIOS)]
            self.load.set_mass(s["mass_kg"] / 2.0)
            self.act_L.max_force = s["force"]
            self.act_R.max_force = s["force"]
            for act in (self.act_L, self.act_R):
                act.stalled            = False
                act.velocity           = 0.0
                act.acceleration       = 0.0
                act.pwm_input          = 0.0
                act.emergency_stopped  = False
            self.ctrl.overload_detected        = False
            self.ctrl.emergency_stop_triggered = False
            self.ctrl.integral_error           = 0.0
            self.ctrl.last_error               = 0.0
            self.ctrl.set_target_position(ACT_STROKE)
            return s           

        # ── bootstrap ──────────────────────────────────────────────────
        cur = _reset_and_load(si)

        last_event_text = ""

        initial_state = {
            "scenario": cur,
            "phase": phase,
            "ext_m": 0.0,
            "stalled": False,
            "overload": False,
            "idx": si,
            "total": len(SCENARIOS),
            "stroke_m": ACT_STROKE,
            "event_text": last_event_text,
        }

        try:
            with create_demo_live(initial_state, refresh_per_second=10) as live:
                with KeyReader() as key_reader:
                    build_demo_scenarios_panel(live, SCENARIOS)
                    while p.isConnected():
                        t0 = time.perf_counter()

                        should_quit = False

                        for ch in key_reader.poll():
                            c = ch.lower()
                            if c == "q":
                                should_quit = True
                                break

                        if should_quit:
                            break
                        
                        # ── 5 physics steps per frame ──────────────────────────
                        for _ in range(DEMO_STEPS):
                            req = self.load.get_required_force(self.act_L.acceleration)
                            pwm = self.ctrl.update(BASE_DT)
                            self.act_L.set_pwm(pwm)
                            self.act_R.set_pwm(pwm)
                            self.act_L.step(BASE_DT, req)
                            self.act_R.step(BASE_DT, req)
                            p.stepSimulation()

                        ext      = (self.act_L.position + self.act_R.position) / 2.0
                        stalled  = self.act_L.stalled or self.act_R.stalled
                        overload = self.ctrl.overload_detected
                        at_tgt   = self.ctrl.at_target()

                        # ── state machine ──────────────────────────────────────
                        if phase == "LIFTING":
                            if at_tgt or stalled or overload:
                                phase      = "HOLD_UP"
                                hold_start = time.perf_counter()
                                tag = "STALLED" if (stalled or overload) else "AT TOP"
                                last_event_text += f"  [{cur['label']:16s}]  {cur['mass_kg']:3} kg  " \
                                                f"{tag:8}  {ext*1000:.1f} mm  " \
                                                f"need {cur['mass_kg']*9.81/2:.0f} N  " \
                                                f"rated {cur['force']} N"
                        elif phase == "HOLD_UP":
                            if time.perf_counter() - hold_start >= HOLD_TOP_S:
                                phase = "LOWERING"
                                # Clear stall so actuator can retract
                                for act in (self.act_L, self.act_R):
                                    act.stalled = False
                                self.ctrl.overload_detected = False
                                self.ctrl.set_target_position(0.0)

                        elif phase == "LOWERING":
                            if ext < 0.003 and at_tgt:
                                phase      = "HOLD_DOWN"
                                hold_start = time.perf_counter()

                        elif phase == "HOLD_DOWN":
                            if time.perf_counter() - hold_start >= HOLD_BTM_S:
                                si  = (si + 1) % len(SCENARIOS)
                                cur = _reset_and_load(si)
                                phase = "LIFTING"
                                last_event_text = f"Scenario {si+1}: {cur['label']} ({cur['mass_kg']} kg)\n"

                        # ── render ─────────────────────────────────────────────
                        if not p.isConnected():
                            break
                        step += 1
                        if step % 2 == 0:
                            try:
                                self._update_visuals(ext, stalled, overload, at_tgt,
                                                    force_per_act=req)
                            except Exception:
                                break   # window closed mid-frame
                        update_demo_live(
                            live,
                            {
                                "scenario": cur,
                                "phase": phase,
                                "ext_m": ext,
                                "stalled": stalled,
                                "overload": overload,
                                "idx": si,
                                "total": len(SCENARIOS),
                                "stroke_m": ACT_STROKE,
                                "event_text": last_event_text,
                            },
                        )

                        # ── real-time pacing (target 100 Hz wall clock) ────────
                        elapsed = time.perf_counter() - t0
                        slack   = BASE_DT - elapsed
                        if slack > 0:
                            time.sleep(slack)

        except KeyboardInterrupt:
            print("\n  Demo stopped.")
        finally:
            try:
                if p.isConnected():
                    p.disconnect()
            except Exception:
                pass
