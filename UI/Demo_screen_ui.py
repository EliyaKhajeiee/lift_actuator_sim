from rich.console import Console
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.prompt import Prompt
from rich.table import Table
from rich.text import Text
from rich.align import Align


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

def build_demo_scenarios_panel(live: Live, scenarios: list[dict]) -> None:
    table = Table(show_header=True, header_style="bold cyan", expand=True)
    table.add_column("#", style="bold white", width=3)
    table.add_column("Scenario", style="bold yellow")
    table.add_column("Mass", justify="right")
    table.add_column("Need", justify="right")
    table.add_column("Rated", justify="right")
    table.add_column("Status", justify="center")

    for i, s in enumerate(scenarios, start=1):
        needed = s["mass_kg"] * 9.81 / 2
        ok = "OK" if needed <= s["force"] else "will STALL"
        status_style = "bold green" if ok == "OK" else "bold red"
        table.add_row(
            str(i),
            s["label"],
            f"{s['mass_kg']:>3} kg",
            f"{needed:.0f} N",
            f"{s['force']} N",
            Text(ok, style=status_style),
        )

    layout["scenarios"].update(Panel(table, title="DEMO SCENARIOS", border_style="bright_blue", padding=(0,1)))
    live.update(layout, refresh=True)

def show_demo_screen() -> None:
    console = Console()
    console.clear()
    global layout
    layout = Layout()
    layout.split_column(
        Layout(name="header", renderable=_show_title_screen("DEMO MODE", "Running automatic demo scenario..."), size=3),
        Layout(name="body", ratio=1),
        Layout(name="footer", size=4),
    )
    layout["body"].split_row(
        Layout(name="left_panel", renderable=None, ratio=1),
        Layout(name="right_panel",renderable= None, ratio=1),
    )
    layout["left_panel"].split_column(
        Layout(name="scenarios", renderable=None, ratio=1),
        Layout(name="note", renderable=_show_note(), ratio=2),
    )
    console.print(layout)

def create_demo_live(initial_state: dict, refresh_per_second: int = 10) -> Live:
    hud_panel = build_demo_live_panel(
        scenario=initial_state["scenario"],
        phase=initial_state["phase"],
        ext_m=initial_state["ext_m"],
        stalled=initial_state["stalled"],
        overload=initial_state["overload"],
        idx=initial_state["idx"],
        total=initial_state["total"],
        stroke_m=initial_state["stroke_m"],
    )
    layout["right_panel"].update(hud_panel)
    event_panel = build_demo_event_panel(initial_state.get("event_text", ""))
    layout["footer"].update(event_panel)
    return Live(layout, refresh_per_second=refresh_per_second, transient=False)

def update_demo_live(live: Live, state: dict) -> None:
    hud_panel  = build_demo_live_panel(
        scenario=state["scenario"],
        phase=state["phase"],
        ext_m=state["ext_m"],
        stalled=state["stalled"],
        overload=state["overload"],
        idx=state["idx"],
        total=state["total"],
        stroke_m=state["stroke_m"],
    )
    layout["right_panel"].update(hud_panel)
    event_panel = build_demo_event_panel(state.get("event_text", ""))
    layout["footer"].update(event_panel)
    live.update(layout, refresh=True)

def build_demo_event_panel(event_text: str) -> Panel:
    body = Text(event_text or "-", style="bold white")
    return Panel(
        body,
        title="EVENT",
        border_style="yellow",
        padding=(0, 1),
    )

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

def _rgb_style(rgb) -> str:
    r = int(max(0, min(255, round(rgb[0] * 255))))
    g = int(max(0, min(255, round(rgb[1] * 255))))
    b = int(max(0, min(255, round(rgb[2] * 255))))
    return f"rgb({r},{g},{b})"


def build_demo_live_panel(
    scenario: dict,
    phase: str,
    ext_m: float,
    stalled: bool,
    overload: bool,
    idx: int,
    total: int,
    stroke_m: float,
):
    mass = scenario["mass_kg"]
    lbs = mass * 2.205
    force = mass * 9.81
    per = force / 2.0
    pct = min(1.0, ext_m / max(stroke_m, 1e-9))
    filled = int(round(pct * 14))
    bar = "\u2588" * filled + "\u2591" * (14 - filled)
    ext_in = ext_m / 0.0254
    force_ok = per <= scenario["force"]

    if stalled or overload:
        ph_str = "\u26a0  STALL / OVERLOAD"
        pc = [1.00, 0.15, 0.10]
    elif phase == "LIFTING":
        ph_str = "\u2191  LIFTING"
        pc = [0.28, 0.86, 0.95]
    elif phase == "HOLD_UP":
        ph_str = "\u25a0  HOLDING AT TOP"
        pc = [0.28, 0.92, 0.44]
    elif phase == "LOWERING":
        ph_str = "\u2193  LOWERING"
        pc = [1.00, 0.64, 0.12]
    else:
        ph_str = "\u22ef  STANDBY"
        pc = [0.50, 0.52, 0.58]

    W = [0.94, 0.94, 0.96]
    DIM = [0.34, 0.36, 0.42]
    YLW = [1.00, 0.84, 0.18]
    GRN = [0.24, 0.90, 0.44]
    RED = [1.00, 0.20, 0.10]
    CYN = [0.24, 0.82, 0.94]

    rows = [
        ("\u2550" * 30, DIM),
        ("  SLING LIFT - AUTO DEMO", [0.92, 0.94, 0.98]),
        (f"  Scenario  {idx + 1} / {total}", W),
        ("\u2550" * 30, DIM),
        (f"  {scenario['label']}", YLW),
        (f"  {mass:.0f} kg  -  {lbs:.0f} lb", W),
        ("\u2500" * 30, DIM),
        (ph_str, pc),
        (f"  [{bar}]  {pct*100:4.1f}%", CYN),
        (f"  Lift  {ext_in:.3f} in  ->  {ext_m*1000:.1f} mm", W),
        ("\u2500" * 30, DIM),
        ("  FORCE CALCULATION", W),
        (f"  Total gravity   {force:>6.0f} N", W),
        (f"  Per actuator    {per:>6.0f} N", W),
        (
            f"  Actuator rated  {scenario['force']:>6} N  "
            f"({'  OK  ' if force_ok else 'EXCEEDED'})",
            GRN if force_ok else RED,
        ),
        ("\u2550" * 30, DIM),
    ]

    body = Text()
    for text, color in rows:
        body.append(text + "\n", style=_rgb_style(color))

    return Panel(
        Align.left(body),
        title="DEMO TELEMETRY",
        border_style="bright_blue",
        padding=(0, 1),
    )
