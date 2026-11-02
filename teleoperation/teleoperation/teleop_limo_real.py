#!/usr/bin/env python3

import threading
import tkinter as tk

from robot_communication.tcp_client import RobotCommunicationClient
from teleoperation.teleop_widget import TeleopWidget


HOST = "127.0.0.1"
PORT = 8765

MAX_SPEED = 0.5
MAX_STEERING = 1.0

BLUE = "#006699"
BLUE_DARK = "#004466"
BG = "#F3F6F8"
WHITE = "#FFFFFF"
TEXT = "#202B36"
MUTED = "#74818D"
RED = "#A62626"
GREEN = "#26734D"
FONT = "DejaVu Sans"


class LimoTeleop:
    def __init__(self, root):
        self.root = root
        self.root.title("ENSEM | LIMO Teleoperation")
        self.root.geometry("600x520")
        self.root.minsize(520, 460)
        self.root.configure(bg=BG)

        self.client = None
        self.connected = False
        self.closing = False

        self.status = tk.StringVar(value="Connecting...")
        self.speed_text = tk.StringVar(value="0.00 m/s")
        self.steering_text = tk.StringVar(value="0.00")

        self._build_header()
        self._connect()

        self.widget = TeleopWidget(
            self.root,
            on_command=self.send_command,
            on_close=self.close,
            max_speed=MAX_SPEED,
            max_steering=MAX_STEERING,
        )
        self.widget.pack(
            fill="both",
            expand=True,
            padx=24,
            pady=16,
        )

        self._build_status()

        if self.connected:
            threading.Thread(
                target=self._receive_loop,
                daemon=True,
            ).start()

        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(150, self.root.focus_force)

    def _build_header(self):
        header = tk.Frame(self.root, bg=BLUE, height=90)
        header.pack(fill="x")
        header.pack_propagate(False)

        left = tk.Frame(header, bg=BLUE)
        left.pack(side="left", padx=22, pady=15)

        tk.Label(
            left,
            text="ENSEM",
            bg=BLUE,
            fg=WHITE,
            font=(FONT, 10, "bold"),
        ).pack(anchor="w")

        tk.Label(
            left,
            text="LIMO TELEOPERATION",
            bg=BLUE,
            fg=WHITE,
            font=(FONT, 19, "bold"),
        ).pack(anchor="w", pady=(3, 0))

        self.connection_badge = tk.Label(
            header,
            text="DISCONNECTED",
            bg=BLUE_DARK,
            fg=WHITE,
            font=(FONT, 9, "bold"),
            padx=12,
            pady=8,
        )
        self.connection_badge.pack(side="right", padx=20)

    def _build_status(self):
        footer = tk.Frame(self.root, bg="#E8EDF1")
        footer.pack(fill="x", side="bottom")

        metrics = tk.Frame(footer, bg="#E8EDF1")
        metrics.pack(side="left", padx=18, pady=8)

        tk.Label(
            metrics,
            textvariable=self.speed_text,
            bg="#E8EDF1",
            fg=TEXT,
            font=(FONT, 9, "bold"),
        ).pack(side="left", padx=(0, 12))

        tk.Label(
            metrics,
            textvariable=self.steering_text,
            bg="#E8EDF1",
            fg=TEXT,
            font=(FONT, 9, "bold"),
        ).pack(side="left")

        tk.Label(
            footer,
            textvariable=self.status,
            bg="#E8EDF1",
            fg=MUTED,
            font=(FONT, 8),
            wraplength=240,
        ).pack(side="right", padx=18, pady=8)

    def _connect(self):
        try:
            self.client = RobotCommunicationClient(
                host=HOST,
                port=PORT,
            )
            self.client.connect()
            self.connected = True

            self.connection_badge.configure(
                text="CONNECTED",
                bg=GREEN,
            )
            self.status.set("Connected to LIMO")

        except (OSError, ConnectionError) as exc:
            self.client = None
            self.connected = False

            self.connection_badge.configure(
                text="DISCONNECTED",
                bg=RED,
            )
            self.status.set(f"Connection failed: {exc}")

    def send_command(self, speed, steering):
        if self.closing:
            return

        self.speed_text.set(f"Speed: {speed:+.2f} m/s")
        self.steering_text.set(f"Steering: {steering:+.2f}")

        if not self.connected or self.client is None:
            return

        try:
            self.client.send_command(
                speed=speed,
                steering=steering,
            )
        except (OSError, ConnectionError) as exc:
            self._connection_lost(str(exc))

    def _receive_loop(self):
        while not self.closing and self.client is not None:
            try:
                message = self.client.receive_message(timeout=1.0)

                if message and message.get("type") == "error":
                    error = message.get("message", "TCP server error")
                    self.root.after(
                        0,
                        self.status.set,
                        error,
                    )

            except (OSError, ConnectionError) as exc:
                if not self.closing:
                    try:
                        self.root.after(
                            0,
                            self._connection_lost,
                            str(exc),
                        )
                    except tk.TclError:
                        pass
                break

    def _connection_lost(self, message):
        if self.closing:
            return

        self.connected = False
        self.connection_badge.configure(
            text="DISCONNECTED",
            bg=RED,
        )
        self.status.set(message)

    def close(self):
        if self.closing:
            return

        self.closing = True

        if hasattr(self, "widget"):
            self.widget.shutdown()

        client = self.client
        self.client = None
        self.connected = False

        if client is not None:
            try:
                client.disconnect(send_stop=True)
            except (OSError, ConnectionError):
                pass

        self.root.destroy()


def main():
    root = tk.Tk()
    LimoTeleop(root)
    root.mainloop()


if __name__ == "__main__":
    main()