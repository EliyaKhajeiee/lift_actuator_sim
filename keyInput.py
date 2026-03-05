import platform
import sys
import select

if platform.system().lower().startswith("win"):
    import msvcrt
else:
    import termios
    import tty

class KeyReader:
    def __init__(self):
        self.is_win = platform.system().lower().startswith("win")
        self.fd = None
        self.old = None
        self.enabled = False

    def __enter__(self):
        if self.is_win:
            self.enabled = True
            return self
        if sys.stdin.isatty():
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            self.enabled = True
        return self

    def __exit__(self, exc_type, exc, tb):
        if (not self.is_win) and self.enabled and self.fd is not None and self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def poll(self):
        if not self.enabled:
            return []
        out = []
        if self.is_win:
            while msvcrt.kbhit():
                ch = msvcrt.getwch()
                out.append(ch)
            return out
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if r:
            ch = sys.stdin.read(1)
            if ch:
                out.append(ch)
        return out