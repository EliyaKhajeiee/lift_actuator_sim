from rich.panel import Panel
from rich.text import Text
from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.align import Align
from rich.table import Table

from core.sim_constants import ACT_STROKE, ACT_MAX_VEL, ACT_EFF

layout = None

def _show_title_screen(title: str, subtitle: str, border_style: str = "bright_blue") -> Panel:
    return Panel.fit(
        Text.assemble(
            (title, "bold cyan"),
            "\n",
            (subtitle, "dim white"),
        ),
        border_style=border_style,
    )

def show_interactive_screen() -> None:
    console = Console()
    console.clear()
    global layout
    layout = Layout()
    layout.split_column(
        Layout(name="header", renderable=_show_title_screen("INTERACTIVE MODE", "Running manual simulation loop..."), size=3),
        Layout(name="body", ratio=1),
        Layout(name="footer", size=4),
    )
    layout["body"].split_row(
        Layout(name="left_panel", renderable=None, ratio=1),
        Layout(name="right_panel",renderable= None, ratio=1, minimum_size=40),
    )
    layout["left_panel"].split_column(
        Layout(name="note", renderable=_show_note(), ratio=2),
        Layout(name="control_panel", renderable=None, ratio=1),
    )
    console.print(layout)

def create_interactive_live(initial_state: dict, refresh_per_second: int = 10) -> Live:
    hud_panel = build_interactive_live_panel(
        user_mass=initial_state["user_mass"],
        ext_m=initial_state["ext_m"],
        pwm=initial_state["pwm"],
        cap_force_each=initial_state["cap_force_each"],
        stalled=initial_state["stalled"],
        overloaded=initial_state["overloaded"],
        at_target=initial_state["at_target"],
        supply_v=initial_state["supply_v"],
        duty_pct=initial_state["duty_pct"],
        target_position=initial_state.get("target_position", 0.0),
    )
    layout["right_panel"].update(hud_panel)
    layout["control_panel"].update(
        build_control_panel(
            {
                "weight": initial_state.get("user_mass", 80.0),
                "target": initial_state.get("target", 0.0),
                "force": initial_state.get("cap_force_each", 1000.0),
                "volts": initial_state.get("supply_v", 12.0),
                "speed": initial_state.get("speed", 1.0),
            }
        )
    )
    return Live(layout, refresh_per_second=refresh_per_second, transient=False)

def update_interactive_live(live: Live, state: dict) -> None:
    hud_panel  = build_interactive_live_panel(
        user_mass=state["user_mass"],
        ext_m=state["ext_m"],
        pwm=state["pwm"],
        cap_force_each=state["cap_force_each"],
        stalled=state["stalled"],
        overloaded=state["overloaded"],
        at_target=state["at_target"],
        supply_v=state["supply_v"],
        duty_pct=state["duty_pct"],
        target_position=state.get("target_position", 0.0),
    )
    layout["right_panel"].update(hud_panel)
    layout["control_panel"].update(
        build_control_panel(
            {
                "weight": state.get("user_mass", 80.0),
                "target": state.get("target", 0.0),
                "force": state.get("cap_force_each", 1000.0),
                "volts": state.get("supply_v", 12.0),
                "speed": state.get("speed", 1.0),
            }
        )
    )
    live.update(layout, refresh=True)

def build_control_panel(state: dict) -> Panel:
    t = Table(show_header=True, header_style="bold cyan", expand=True)
    t.add_column("Control")
    t.add_column("Value(control keys)", justify="right")
    t.add_row("User Weight (kg)", f"{state['weight']:.1f}   (j/k)")
    t.add_row("Lift Target (0-1)", f"{state['target']:.3f}   (n/m)")
    t.add_row("Actuator Force (N)", f"{state['force']:.0f}   (u/i)")
    t.add_row("Supply Voltage (V)", f"{state['volts']:.0f}   (v/b)")
    t.add_row("Sim Speed", f"{state['speed']:.2f}   (1/2)")
    t.add_row("quit", "Press [bold yellow]q[/bold yellow] to quit")
    return Panel(t, title="CONTROL PANEL", border_style="green")


def build_interactive_live_panel(
    user_mass: float,
    ext_m: float,
    pwm: float,
    cap_force_each: float,
    stalled: bool,
    overloaded: bool,
    at_target: bool,
    supply_v: float,
    duty_pct: float,
    target_position: float = 0.0,
):
    GRAVITY = 9.81
    SAFETY_F = 2.0

    total_grav = user_mass * GRAVITY
    per_act_req = total_grav / 2.0
    per_act_des = per_act_req * SAFETY_F
    pct_cap = min(100.0, per_act_req / max(cap_force_each, 1.0) * 100.0)

    std_ratings = [200, 350, 500, 750, 1000, 1500, 2000, 3000]
    rec_rating = next((r for r in std_ratings if r >= per_act_des), 3000)

    ext_in = ext_m / 0.0254
    stroke_in = ACT_STROKE / 0.0254

    pct_lift = min(1.0, ext_m / max(ACT_STROKE, 1e-9))
    filled = int(round(pct_lift * 12))
    bar = "\u2588" * filled + "\u2591" * (12 - filled)
    bar_label = f"  [{bar}]  {pct_lift*100:4.1f}%"

    power_w = per_act_req * ACT_MAX_VEL / ACT_EFF
    current_a = power_w / max(supply_v, 1.0)
    total_i = current_a * 2
    wire_i = total_i * 2.0
    if wire_i < 3.0:
        awg = "AWG 20"
    elif wire_i < 5.0:
        awg = "AWG 18"
    elif wire_i < 10.0:
        awg = "AWG 16"
    elif wire_i < 15.0:
        awg = "AWG 14"
    elif wire_i < 20.0:
        awg = "AWG 12"
    else:
        awg = "AWG 10"

    W = [0.94, 0.94, 0.96]
    DIM = [0.34, 0.36, 0.42]
    YLW = [1.00, 0.84, 0.18]
    GRN = [0.24, 0.90, 0.44]
    RED = [1.00, 0.20, 0.10]
    CYN = [0.24, 0.82, 0.94]
    force_c = RED if pct_cap > 90 else (YLW if pct_cap > 70 else W)
    pwm_c = RED if stalled else GRN
    duty_c = RED if duty_pct > 25 else (YLW if duty_pct > 18 else GRN)

    if stalled or overloaded:
        status, sc = "OVERLOADED  -  ACTUATORS STALLED", RED
    elif at_target and ext_m > 0.003:
        status, sc = "AT TARGET  -  Holding", GRN
    elif target_position > 0.003 and ext_m < target_position - 0.003:
        status, sc = "LIFTING  ...", [0.35, 0.78, 1.00]
    elif target_position < 0.003 and ext_m > 0.003:
        status, sc = "LOWERING  ...", [1.00, 0.65, 0.12]
    else:
        status, sc = "STANDBY  -  Ready", [0.58, 0.60, 0.65]

    rows = [
        ("\u2550" * 36, DIM),
        ("  SLING LIFT ACTUATOR SYSTEM", [0.92, 0.94, 0.98]),
        ("\u2550" * 36, DIM),
        (status, sc),
        ("\u2500" * 36, DIM),
        ("LIFT PROGRESS", W),
        (bar_label, CYN),
        (f"  {ext_in:.3f} in  ->  {ext_m*1000:.1f} mm  of {stroke_in:.0f} in stroke", W),
        ("\u2500" * 36, DIM),
        ("LOAD ANALYSIS", W),
        (f"  User weight    {user_mass:>5.0f} kg  ({user_mass*2.205:.0f} lb)", W),
        (f"  Gravity force  {total_grav:>5.0f} N  total", force_c),
        (f"  Per actuator   {per_act_req:>5.0f} N  ({pct_cap:.0f}% of rated {cap_force_each:.0f} N)", force_c),
        (f"  2x safety      {per_act_des:>5.0f} N  -> specify >= {rec_rating} N", YLW),
        ("\u2500" * 36, DIM),
        (f"ELECTRICAL  -  {supply_v:.0f} V DC", W),
        (f"  Per actuator   {power_w:>5.1f} W   {current_a:.2f} A", W),
        (f"  Both actuators {power_w*2:>5.1f} W   {total_i:.2f} A", W),
        (f"  Min wire gauge  {awg}  (60 C, 2x safety)", CYN),
        (f"  Live duty cycle {duty_pct:>4.1f}%  (spec <= 25%)", duty_c),
        ("\u2500" * 36, DIM),
        ("PROCUREMENT SPEC", GRN),
        (f"  Force >= {rec_rating} N      Stroke {stroke_in:.0f} in     {supply_v:.0f} V DC", GRN),
        (f"  Speed >= {ACT_MAX_VEL*1000:.0f} mm/s    Enclosure IP54 or better", GRN),
        ("  e.g. Progressive Automations PA-14", DIM),
        ("       Firgelli Automations FA-PO series", DIM),
        ("\u2500" * 36, DIM),
        (f"  PWM  {pwm:>+.3f}     {'STALLED' if stalled else 'running'}", pwm_c),
    ]

    body = Text()
    for text, color in rows:
        body.append(text + "\n", style=_rgb_style(color))

    return Panel(
        Align.left(body),
        title="INTERACTIVE TELEMETRY",
        border_style="bright_blue",
        padding=(0, 1),
    )


def build_interactive_live_pannel(**kwargs):
    """Backward-compatible alias for typo'd function name."""
    return build_interactive_live_panel(**kwargs)

def _rgb_style(rgb) -> str:
    r = int(max(0, min(255, round(rgb[0] * 255))))
    g = int(max(0, min(255, round(rgb[1] * 255))))
    b = int(max(0, min(255, round(rgb[2] * 255))))
    return f"rgb({r},{g},{b})"

def _show_note() -> Panel:
    return Panel.fit(
        Text(
        "\n"
        + "=" * 64 + "\n"
        + "  WHEELCHAIR SLING LIFT — 3D Demo\n"
        + "=" * 64 + "\n"
        + "  Two side-mounted linear actuators lift the sling.\n"
        + "\n"
        + "  Sliders (left panel):\n"
        + "    User Weight   — sets load; HUD shows required force\n"
        + "    Lift Target   — 0 = down, 1 = full 4-inch lift\n"
        + "    Max Force     — reduce to see actuator stall (red)\n"
        + "    Supply Volt   — 12 V or 24 V (affects current calc)\n"
        + "    Sim Speed     — 0.25 = slow-motion, 1 = real-time, 4 = fast\n"
        + "\n"
        + "  Ruler (right side): shows 0–100% stroke with live marker.\n"
        + "  HUD: load, progress bar, electrical, wire gauge, buy spec.\n",
        style="white",
    ),
    border_style="bright_blue",
    )





'''
def _update_hud(self, user_mass, ext_m, pwm, cap_force_each,
                    stalled, overloaded, at_target, supply_v, duty_pct):
        """Engineering panel — everything needed to buy and build this."""
        GRAVITY     = 9.81
        SAFETY_F    = 2.0
        total_grav  = user_mass * GRAVITY
        per_act_req = total_grav / 2.0
        per_act_des = per_act_req * SAFETY_F
        pct_cap     = min(100.0, per_act_req / max(cap_force_each, 1.0) * 100.0)

        # Recommended actuator rating (next standard size above design load)
        std_ratings = [200, 350, 500, 750, 1000, 1500, 2000, 3000]
        rec_rating  = next((r for r in std_ratings if r >= per_act_des), 3000)

        # Performance
        ext_in       = ext_m / 0.0254
        stroke_in    = ACT_STROKE / 0.0254
        time_to_full = ACT_STROKE / ACT_MAX_VEL

        # Lift progress bar
        pct_lift  = min(1.0, ext_m / max(ACT_STROKE, 1e-9))
        filled    = int(round(pct_lift * 12))
        bar       = "\u2588" * filled + "\u2591" * (12 - filled)
        bar_label = f"  [{bar}]  {pct_lift*100:4.1f}%"

        # Electrical (at rated load)
        power_w   = per_act_req * ACT_MAX_VEL / ACT_EFF   # W per actuator
        current_a = power_w / max(supply_v, 1.0)           # A per actuator
        total_i   = current_a * 2                          # both actuators

        # Wire gauge recommendation (2× for safety margin on continuous run)
        wire_i = total_i * 2.0
        if   wire_i <  3.0: awg = "AWG 20"
        elif wire_i <  5.0: awg = "AWG 18"
        elif wire_i < 10.0: awg = "AWG 16"
        elif wire_i < 15.0: awg = "AWG 14"
        elif wire_i < 20.0: awg = "AWG 12"
        else:                awg = "AWG 10"

        # Status
        if stalled or overloaded:
            status, sc = "OVERLOADED  —  ACTUATORS STALLED", [1.00, 0.12, 0.08]
        elif at_target and ext_m > 0.003:
            status, sc = "AT TARGET  —  Holding",            [0.18, 0.90, 0.36]
        elif self.ctrl.target_position > 0.003 and \
                ext_m < self.ctrl.target_position - 0.003:
            status, sc = "LIFTING  …",                       [0.35, 0.78, 1.00]
        elif self.ctrl.target_position < 0.003 and ext_m > 0.003:
            status, sc = "LOWERING  …",                      [1.00, 0.65, 0.12]
        else:
            status, sc = "STANDBY  —  Ready",                [0.58, 0.60, 0.65]

        W   = [0.93, 0.93, 0.93]
        DIM = [0.42, 0.44, 0.48]
        YLW = [1.00, 0.82, 0.22]
        RED = [1.00, 0.32, 0.12]
        GRN = [0.30, 0.92, 0.45]
        CYN = [0.30, 0.88, 0.95]
        force_c = RED if pct_cap > 90 else (YLW if pct_cap > 70 else W)
        pwm_c   = RED if stalled else GRN
        duty_c  = RED if duty_pct > 25 else (YLW if duty_pct > 18 else GRN)

        tx = 0.56
        z0 = SEAT_H + ACT_BODY_H + ACT_SHAFT_H + ACT_STROKE + 0.38

        lines = [
            # ── header ────────────────────────────────────────────────
            ("\u2550" * 36,
             DIM,  0.80, z0 + 0.14),
            ("  SLING LIFT ACTUATOR SYSTEM",
             [0.92, 0.94, 0.98], 1.15, z0 + 0.04),
            ("\u2550" * 36,
             DIM,  0.80, z0 - 0.06),
            # ── operational status ────────────────────────────────────
            (status,
             sc,   1.30, z0 - 0.17),
            ("\u2500" * 36,
             DIM,  0.78, z0 - 0.28),
            # ── lift progress ─────────────────────────────────────────
            ("LIFT PROGRESS",
             W,    0.95, z0 - 0.39),
            (bar_label,
             CYN,  1.05, z0 - 0.49),
            (f"  {ext_in:.3f} in  \u2192  {ext_m*1000:.1f} mm  "
             f"of {stroke_in:.0f} in stroke",
             W,    0.95, z0 - 0.59),
            ("\u2500" * 36,
             DIM,  0.78, z0 - 0.69),
            # ── load analysis ─────────────────────────────────────────
            ("LOAD ANALYSIS",
             W,    0.95, z0 - 0.80),
            (f"  User weight    {user_mass:>5.0f} kg  "
             f"({user_mass*2.205:.0f} lb)",
             W,    1.00, z0 - 0.90),
            (f"  Gravity force  {total_grav:>5.0f} N  total",
             force_c, 1.00, z0 - 1.00),
            (f"  Per actuator   {per_act_req:>5.0f} N  "
             f"({pct_cap:.0f}% of rated {cap_force_each:.0f} N)",
             force_c, 1.00, z0 - 1.10),
            (f"  2\u00d7 safety     {per_act_des:>5.0f} N  "
             f"\u2192 specify \u2265 {rec_rating} N",
             YLW,  1.00, z0 - 1.20),
            ("\u2500" * 36,
             DIM,  0.78, z0 - 1.30),
            # ── electrical ────────────────────────────────────────────
            (f"ELECTRICAL  \u2014  {supply_v:.0f} V DC",
             W,    0.95, z0 - 1.41),
            (f"  Per actuator   {power_w:>5.1f} W   {current_a:.2f} A",
             W,    1.00, z0 - 1.51),
            (f"  Both actuators {power_w*2:>5.1f} W   {total_i:.2f} A",
             W,    1.00, z0 - 1.61),
            (f"  Min wire gauge  {awg}  (60 \u00b0C, 2\u00d7 safety)",
             CYN,  1.00, z0 - 1.71),
            (f"  Live duty cycle {duty_pct:>4.1f}%  (spec \u2264 25%)",
             duty_c, 1.00, z0 - 1.81),
            ("\u2500" * 36,
             DIM,  0.78, z0 - 1.91),
            # ── procurement spec ──────────────────────────────────────
            ("PROCUREMENT SPEC",
             GRN,  0.95, z0 - 2.02),
            (f"  Force  \u2265 {rec_rating} N      "
             f"Stroke  {stroke_in:.0f} in     {supply_v:.0f} V DC",
             GRN,  1.00, z0 - 2.12),
            (f"  Speed  \u2265 {ACT_MAX_VEL*1000:.0f} mm/s    "
             f"Enclosure  IP54 or better",
             GRN,  1.00, z0 - 2.22),
            (f"  e.g.  Progressive Automations PA-14",
             DIM,  0.88, z0 - 2.32),
            (f"        Firgelli Automations FA-PO series",
             DIM,  0.88, z0 - 2.41),
            ("\u2500" * 36,
             DIM,  0.78, z0 - 2.50),
            # ── live telemetry ────────────────────────────────────────
            (f"  PWM  {pwm:>+.3f}     "
             f"{'STALLED' if stalled else 'running'}",
             pwm_c, 1.00, z0 - 2.60),
        ]

        for key, (text, color, size, z) in enumerate(lines):
            kw = dict(text=text, textPosition=[tx, 0, z],
                      textColorRGB=color, textSize=size, lifeTime=0)
            if key in self._txt:
                kw['replaceItemUniqueId'] = self._txt[key]
            self._txt[key] = p.addUserDebugText(**kw)
'''
