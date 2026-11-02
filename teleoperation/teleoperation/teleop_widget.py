import tkinter as tk


BLUE = "#006699"
BLUE_DARK = "#004466"
WHITE = "#FFFFFF"
TEXT = "#202B36"
MUTED = "#74818D"
BORDER = "#DFE6EB"
BG = "#F0F4F7"
RED = "#A62626"
FONT = "DejaVu Sans"


class TeleopWidget(tk.Frame):
    """Reusable Tkinter widget for AZERTY LIMO teleoperation."""

    def __init__(
        self,
        parent,
        on_command,
        on_close=None,
        max_speed=0.5,
        max_steering=1.0,
        period_ms=50,
    ):
        super().__init__(
            parent,
            bg=WHITE,
            highlightbackground=BORDER,
            highlightthickness=1,
        )

        self.on_command = on_command
        self.on_close = on_close
        self.max_speed = max_speed
        self.max_steering = max_steering
        self.period_ms = period_ms

        self.pressed = set()
        self.speed = 0.0
        self.steering = 0.0
        self.closed = False
        self.key_widgets = {}

        self._build_ui()

        # Keyboard events are handled by the containing window.
        self.bind_all("<KeyPress>", self._on_key_press, add="+")
        self.bind_all("<KeyRelease>", self._on_key_release, add="+")
        self.winfo_toplevel().bind(
            "<FocusOut>", self._on_focus_out, add="+"
        )

        self.after(self.period_ms, self._publish_command)

    def _build_ui(self):
        tk.Label(
            self,
            text="CONTROLS: AZERTY KEYBOARD",
            bg=WHITE,
            fg=MUTED,
            font=(FONT, 9, "bold"),
        ).pack(anchor="w", padx=18, pady=(18, 12))

        pad = tk.Frame(self, bg=WHITE)
        pad.pack(expand=True, pady=(0, 12))

        for col in range(3):
            pad.columnconfigure(col, minsize=82)

        self._add_key(pad, "Z", "FORWARD", 0, 1)
        self._add_key(pad, "Q", "LEFT", 1, 0)
        self._add_key(pad, "S", "BACKWARD", 1, 1)
        self._add_key(pad, "D", "RIGHT", 1, 2)

        self.space_widget = tk.Label(
            pad,
            text="SPACEBAR — STOP",
            bg=BG,
            fg=TEXT,
            font=(FONT, 11, "bold"),
            width=24,
            pady=10,
            highlightbackground=BORDER,
            highlightthickness=1,
        )
        self.space_widget.grid(
            row=2,
            column=0,
            columnspan=3,
            padx=5,
            pady=(12, 0),
            sticky="ew",
        )

        tk.Label(
            self,
            text="ESC — CLOSE WINDOW",
            bg=WHITE,
            fg=MUTED,
            font=(FONT, 8, "bold"),
        ).pack(pady=(0, 16))

    def _add_key(self, parent, key, label, row, col):
        frame = tk.Frame(parent, bg=WHITE)
        frame.grid(row=row, column=col, padx=5, pady=5)

        widget = tk.Label(
            frame,
            text=key,
            bg=BG,
            fg=TEXT,
            font=(FONT, 18, "bold"),
            width=3,
            pady=8,
            highlightbackground=BORDER,
            highlightthickness=1,
        )
        widget.pack()

        tk.Label(
            frame,
            text=label,
            bg=WHITE,
            fg=MUTED,
            font=(FONT, 8, "bold"),
        ).pack(pady=(4, 0))

        self.key_widgets[key.lower()] = widget

    def _on_key_press(self, event):
        if self.closed:
            return

        key = event.keysym.lower()

        if key in ("z", "q", "s", "d"):
            self.pressed.add(key)
            self._update_command()
            self._refresh_keys()

        elif key == "space":
            self.pressed.clear()
            self.speed = 0.0
            self.steering = 0.0
            self._refresh_keys()
            self._send_stop()
            self.space_widget.configure(bg=RED, fg=WHITE)

        elif key == "escape" and self.on_close:
            self.on_close()

    def _on_key_release(self, event):
        if self.closed:
            return

        key = event.keysym.lower()
        self.pressed.discard(key)

        if key == "space":
            self.space_widget.configure(bg=BG, fg=TEXT)

        self._update_command()
        self._refresh_keys()

    def _on_focus_out(self, _event):
        if self.closed:
            return

        self.pressed.clear()
        self.speed = 0.0
        self.steering = 0.0
        self._refresh_keys()
        self.space_widget.configure(bg=BG, fg=TEXT)
        self._send_stop()

    def _update_command(self):
        self.speed = 0.0
        self.steering = 0.0

        if "z" in self.pressed and "s" not in self.pressed:
            self.speed = self.max_speed
        elif "s" in self.pressed and "z" not in self.pressed:
            self.speed = -self.max_speed

        if "q" in self.pressed and "d" not in self.pressed:
            self.steering = self.max_steering
        elif "d" in self.pressed and "q" not in self.pressed:
            self.steering = -self.max_steering

    def _refresh_keys(self):
        for key, widget in self.key_widgets.items():
            active = key in self.pressed
            widget.configure(
                bg=BLUE if active else BG,
                fg=WHITE if active else TEXT,
                highlightbackground=BLUE if active else BORDER,
            )

    def _send_stop(self):
        try:
            self.on_command(0.0, 0.0)
        except Exception as exc:
            print(f"Teleoperation stop error: {exc}")

    def _publish_command(self):
        if self.closed:
            return

        try:
            self.on_command(self.speed, self.steering)
        except Exception as exc:
            print(f"Teleoperation command error: {exc}")

        self.after(self.period_ms, self._publish_command)

    def get_command(self):
        """Return the current (speed, steering) command."""
        return self.speed, self.steering

    def shutdown(self):
        """Stop keyboard handling and periodic callbacks."""
        self.closed = True
        self.pressed.clear()
        self.speed = 0.0
        self.steering = 0.0