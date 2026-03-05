from rich.console import Console
from rich.panel import Panel
from rich.prompt import Prompt
from rich.table import Table
from rich.text import Text
from pyfiglet import Figlet

from .Demo_screen_ui import show_demo_screen
from .interactive_screen_ui import show_interactive_screen


MODES = ("demo", "interactive", "serial", "quit")


def show_title(console: Console) -> None:
    figlet = Figlet(font="slant")
    title = Text(figlet.renderText("AbleWare Simulator test"), style="bold cyan", justify="center")
    subtitle = Text(
        "Eliya Khajeie, Sahil Gupta, Linh My On, Jaehyung Choi",
        style="dim white",
        justify="center",
    )
    console.print(Panel.fit(Text.assemble(title, "\n", subtitle), border_style="bright_blue"))
    console.print(Text("  Select run mode:\n", style="white", justify="center"))


def show_mode_table(console: Console) -> None:
    table = Table(title="Available Modes", show_header=True, header_style="bold magenta")
    table.add_column("Mode", style="bold green")
    table.add_column("Description", style="white")
    table.add_row("demo", "Auto scenario demo")
    table.add_row("interactive", "Manual control loop")
    table.add_row("serial", "Serial bridge control")
    table.add_row("quit", "Exit the program")
    console.print(table)


def prompt_mode(default: str = "demo") -> str:
    console = Console()
    show_title(console)
    show_mode_table(console)
    return Prompt.ask(
        "[bold yellow]Choose mode[/bold yellow]",
        choices=list(MODES),
        default=default,
    )


def _show_title_screen(console: Console, title: str, subtitle: str, border_style: str = "bright_blue") -> None:
    console.clear()
    console.print(
        Panel.fit(
            Text.assemble((title, "bold cyan"), "\n", (subtitle, "dim white")),
            border_style=border_style,
        )
    )



def show_serial_screen() -> None:
    _show_title_screen(Console(), "SERIAL MODE", "Starting serial bridge...")


def show_quit_screen() -> None:
    _show_title_screen(Console(), "EXIT", "Closing simulator...", border_style="grey50")


def show_mode_screen(mode: str) -> None:
    screens = {
        "demo": show_demo_screen,
        "interactive": show_interactive_screen,
        "serial": show_serial_screen,
        "quit": show_quit_screen,
    }
    screens.get(mode, lambda: _show_title_screen(Console(), mode.upper(), "Starting..."))()


if __name__ == "__main__":
    selected_mode = prompt_mode()
    Console().print(f"[bold green]Selected:[/bold green] {selected_mode}")
