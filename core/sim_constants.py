# ── Geometry constants (metres, Z-up) ─────────────────────────────────────

SEAT_H      = 0.480   # floor → seat top
SEAT_W      = 0.460   # left/right
SEAT_D      = 0.410   # front/back
CUSHION_T   = 0.050

BACK_H      = 0.450   # backrest height above seat
WHEEL_R     = 0.305   # rear wheel radius (24-inch)
WHEEL_T     = 0.038
CASTER_R    = 0.076
CASTER_T    = 0.024

SLING_W     = 0.370
SLING_D     = 0.350
SLING_T     = 0.018

# ── Actuator: real-world spec (PA-14 class, 1000 N, 4-inch stroke, 12/24 V)
ACT_STROKE  = 0.1016   # 4-inch stroke  — most common practical size
ACT_R       = 0.028    # outer body radius (~56 mm OD tube)
ACT_BODY_H  = 0.200    # motor housing height
ACT_SHAFT_H = 0.145    # visible shaft at full extension; > stroke for overlap
ACT_SIDE_Y  = SEAT_W / 2 + 0.045   # Y mount: outside seat, inside wheels
ACT_X       = 0.0      # centred front-back

# Rated physics (PA-14 1000 N @ 12 V)
ACT_MAX_VEL  = 0.010   # m/s  (10 mm/s at rated load — datasheet typical)
ACT_MAX_ACC  = 0.040   # m/s²
ACT_RAMP_T   = 0.60    # s    (slow worm-gear ramp)
ACT_EFF      = 0.70    # drivetrain efficiency

# Arm geometry (extends inward from shaft top over the sling)
ARM_INNER_Y = SLING_W / 2 + 0.008
ARM_R       = 0.012

# Sling-strap attachment positions
TIE_X_F     =  SLING_D / 2 - 0.045
TIE_X_R     = -SLING_D / 2 + 0.045
TIE_W       = 0.032    # nylon webbing strap width (32 mm)
TIE_T       = 0.005    # strap thickness

TUBE_R      = 0.011    # wheelchair frame tube radius

# Sling tilt: rear (butt side) rises fully; front (thigh side) rises TILT_RATIO × as much
# Creates forward nose-down tilt so butt clears the seat for pants-dressing
TILT_RATIO  = 0.48     # 0 = full tilt, 1 = flat; 0.48 ≈ 11° at full stroke

ACT_CAP_H   = 0.016   # housing top end-cap thickness
ACT_XBRACE_Z = SEAT_H + ACT_BODY_H * 0.42   # cross-brace height on housing

SLING_STRAP_W = 0.042  # nylon strap width (42 mm)
SLING_RAIL_W  = 0.028  # side rail width  (28 mm)

RULER_X     = ACT_SIDE_Y + 0.095   # X position of stroke ruler

ACTUATOR_STROKE = ACT_STROKE   # back-compat alias for tests
# ── Colour palette ─────────────────────────────────────────────────────────

C_FRAME_DK  = [0.12, 0.13, 0.15, 1.0]   # near-black structural steel
C_FRAME_MD  = [0.26, 0.28, 0.32, 1.0]   # gunmetal mid steel
C_FRAME_LT  = [0.60, 0.64, 0.70, 1.0]   # polished chrome highlight

C_TYRE      = [0.06, 0.06, 0.07, 1.0]   # near-black rubber
C_RIM       = [0.74, 0.76, 0.80, 1.0]   # machined-aluminum rim
C_SPOKE     = [0.48, 0.50, 0.55, 1.0]   # spoke steel
C_HUB       = [0.30, 0.32, 0.36, 1.0]   # hub steel

C_CUSHION   = [0.06, 0.08, 0.25, 0.97]  # midnight-navy seat cushion
C_SLING_DN  = [0.05, 0.32, 0.78, 1.0]   # vivid medical blue (at rest)
C_SLING_UP  = [0.06, 0.78, 0.44, 1.0]   # bright medical green (lifted)
C_SLING_EDG = [0.03, 0.20, 0.52, 1.0]   # deep-blue rail edge

C_ACT_IDLE  = [0.32, 0.34, 0.38, 1.0]   # idle rod (dark steel)
C_ACT_OK    = [0.10, 0.84, 0.40, 1.0]   # bright operational green
C_ACT_WARN  = [0.96, 0.70, 0.08, 1.0]   # amber warning
C_ACT_DEAD  = [0.92, 0.12, 0.10, 1.0]   # bright fault red

C_ARM       = [0.56, 0.59, 0.64, 1.0]   # yoke-arm chrome
C_TIE       = [0.06, 0.28, 0.76, 1.0]   # vivid nylon-webbing blue
C_TIE_EDGE  = [0.04, 0.16, 0.50, 1.0]   # strap border dark blue

C_ACT_HOUSE = [0.80, 0.82, 0.86, 1.0]   # brushed-aluminum actuator tube
C_ACT_MOTOR = [0.18, 0.19, 0.22, 1.0]   # dark-gunmetal motor/gearbox body
C_ACT_ROD   = [0.30, 0.32, 0.36, 1.0]   # polished steel inner rod
C_CAP       = [0.16, 0.17, 0.20, 1.0]   # end-cap / clevis near-black steel
C_CABLE     = [0.04, 0.04, 0.05, 1.0]   # wiring harness near-black

C_RULER     = [0.35, 0.80, 0.42]         # green stroke ruler   (3-ch RGB — debug lines)
C_LABEL_Y   = [0.90, 0.85, 0.25]         # amber annotation text (3-ch RGB — debug text)

C_FLOOR_A   = [0.08, 0.09, 0.11, 1.0]   # near-black charcoal polished floor
C_FLOOR_B   = [0.17, 0.19, 0.22, 1.0]   # subtle grid lines