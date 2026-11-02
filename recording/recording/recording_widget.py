
import tkinter as tk
from tkinter import messagebox

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


BLUE = "#006699"
BLUE_DARK = "#004466"
WHITE = "#FFFFFF"
TEXT = "#202B36"
MUTED = "#74818D"
BORDER = "#DFE6EB"
BG = "#F0F4F7"
RED = "#A62626"
FONT = "DejaVu Sans"


class RecordingWidget:
    """Tkinter interface for controlling ROS 2 bag recording."""

    def __init__(self, node):
        self.node = node
        self.pending_future = None
        self.closed = False

        self.start_client = node.create_client(
            Trigger, "/start_recording"
        )
        self.stop_client = node.create_client(
            Trigger, "/stop_recording"
        )

        self.root = tk.Tk()
        self.root.title("LIMO — Recording")
        self.root.configure(bg=BG)
        self.root.resizable(False, False)

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(50, self._poll_ros)

    def _build_ui(self):
        # Main white panel
        panel = tk.Frame(
            self.root,
            bg=WHITE,
            highlightbackground=BORDER,
            highlightthickness=1,
        )
        panel.pack(padx=18, pady=18)

        # Header
        tk.Label(
            panel,
            text="DATA RECORDING",
            bg=WHITE,
            fg=MUTED,
            font=(FONT, 9, "bold"),
        ).pack(anchor="w", padx=20, pady=(18, 4))

        tk.Label(
            panel,
            text="LIMO",
            bg=WHITE,
            fg=TEXT,
            font=(FONT, 20, "bold"),
        ).pack(anchor="w", padx=20)

        tk.Label(
            panel,
            text="ROS 2 BAG",
            bg=WHITE,
            fg=MUTED,
            font=(FONT, 9, "bold"),
        ).pack(anchor="w", padx=20, pady=(0, 16))

        # Status area
        status_frame = tk.Frame(
            panel,
            bg=BG,
            highlightbackground=BORDER,
            highlightthickness=1,
        )
        status_frame.pack(fill="x", padx=20, pady=(0, 16))

        self.status_indicator = tk.Label(
            status_frame,
            text="●",
            bg=BG,
            fg=MUTED,
            font=(FONT, 13, "bold"),
        )
        self.status_indicator.pack(side="left", padx=(12, 8), pady=12)

        self.status_label = tk.Label(
            status_frame,
            text="Ready",
            bg=BG,
            fg=TEXT,
            font=(FONT, 10, "bold"),
            anchor="w",
        )
        self.status_label.pack(
            side="left", fill="x", expand=True, pady=12
        )

        # Controls
        self.start_button = tk.Button(
            panel,
            text="START RECORDING",
            command=self._start_recording,
            bg=BLUE,
            fg=WHITE,
            activebackground=BLUE_DARK,
            activeforeground=WHITE,
            disabledforeground=WHITE,
            font=(FONT, 10, "bold"),
            relief="flat",
            bd=0,
            cursor="hand2",
            width=25,
            pady=11,
        )
        self.start_button.pack(padx=20, pady=(0, 8), fill="x")

        self.stop_button = tk.Button(
            panel,
            text="STOP RECORDING",
            command=self._stop_recording,
            bg=BG,
            fg=RED,
            activebackground=BORDER,
            activeforeground=RED,
            disabledforeground=MUTED,
            font=(FONT, 10, "bold"),
            relief="flat",
            bd=0,
            cursor="hand2",
            width=25,
            pady=11,
            highlightbackground=BORDER,
            highlightthickness=1,
        )
        self.stop_button.pack(padx=20, pady=(0, 16), fill="x")

        tk.Label(
            panel,
            text="ESC — CLOSE WINDOW",
            bg=WHITE,
            fg=MUTED,
            font=(FONT, 8, "bold"),
        ).pack(pady=(0, 14))

        self.root.bind("<Escape>", lambda _event: self.close())

    def _set_status(self, text, color=MUTED):
        self.status_label.config(text=text, fg=TEXT)
        self.status_indicator.config(fg=color)

    def _start_recording(self):
        self._call_service(
            self.start_client,
            "Starting recording...",
            "●",
        )

    def _stop_recording(self):
        self._call_service(
            self.stop_client,
            "Stopping recording...",
            "●",
        )

    def _call_service(self, client, status, _indicator):
        if self.pending_future is not None:
            return

        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=0.0):
                messagebox.showerror(
                    "Recorder unavailable",
                    "Start the recorder node first.",
                    parent=self.root,
                )
                return

        self.pending_future = client.call_async(
            Trigger.Request()
        )

        self.start_button.config(state="disabled")
        self.stop_button.config(state="disabled")
        self._set_status(status, BLUE)

    def _poll_ros(self):
        if self.closed or not rclpy.ok():
            return

        rclpy.spin_once(self.node, timeout_sec=0.0)

        if (
            self.pending_future is not None
            and self.pending_future.done()
        ):
            try:
                response = self.pending_future.result()

                if response.success:
                    self._set_status(
                        response.message or "Command successful",
                        BLUE,
                    )
                else:
                    self._set_status(
                        response.message or "Command failed",
                        RED,
                    )
                    messagebox.showwarning(
                        "Recording",
                        response.message,
                        parent=self.root,
                    )

            except Exception as error:
                self._set_status("Service call failed", RED)
                messagebox.showerror(
                    "ROS 2 error",
                    str(error),
                    parent=self.root,
                )

            finally:
                self.pending_future = None
                if not self.closed:
                    self.start_button.config(state="normal")
                    self.stop_button.config(state="normal")

        self.root.after(50, self._poll_ros)

    def close(self):
        if self.closed:
            return

        self.closed = True
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = Node("recording_widget")

    try:
        app = RecordingWidget(node)
        app.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
