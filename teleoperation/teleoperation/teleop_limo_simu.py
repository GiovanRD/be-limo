
#!/usr/bin/env python3

"""Tkinter keyboard teleoperation for the simulated AgileX LIMO."""

import math
import tkinter as tk

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from teleoperation.teleop_widget import TeleopWidget


MAX_SPEED = 0.75
MAX_STEERING = math.pi / 6
PERIOD_MS = 20  # 50 Hz

BLUE = "#006699"
BLUE_DARK = "#004466"
BG = "#F3F6F8"
WHITE = "#FFFFFF"
TEXT = "#202B36"
MUTED = "#74818D"
RED = "#A62626"
GREEN = "#26734D"
FONT = "DejaVu Sans"


class LimoTeleopNode(Node):
    """ROS 2 interface for simulated LIMO teleoperation."""

    def __init__(self):
        super().__init__("teleop_limo_simu")

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        self.get_logger().info("Simulated LIMO teleoperation ready")

    def publish_command(self, speed, steering):
        """Publish the current speed and steering command."""

        msg = Twist()
        msg.linear.x = float(speed)

        # Preserve the steering convention of the original teleop.
        msg.angular.z = float(steering)
        if speed < 0.0:
            msg.angular.z = -msg.angular.z

        self.cmd_pub.publish(msg)


class LimoTeleopSimulation:
    """Tkinter interface for simulated LIMO teleoperation."""

    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.closing = False

        self.root.title("ENSEM | LIMO Simulation Teleoperation")
        self.root.geometry("600x520")
        self.root.minsize(520, 460)
        self.root.configure(bg=BG)

        self.speed_text = tk.StringVar(value="Speed: +0.00 m/s")
        self.steering_text = tk.StringVar(value="Steering: +0.00 rad")

        self._build_header()

        self.widget = TeleopWidget(
            self.root,
            on_command=self.node.publish_command,
            on_close=self.close,
            max_speed=MAX_SPEED,
            max_steering=MAX_STEERING,
            period_ms=PERIOD_MS,
        )
        self.widget.pack(
            fill="both",
            expand=True,
            padx=24,
            pady=16,
        )

        self._build_status()

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
            text="LIMO SIMULATION",
            bg=BLUE,
            fg=WHITE,
            font=(FONT, 19, "bold"),
        ).pack(anchor="w", pady=(3, 0))

        tk.Label(
            header,
            text="ROS 2",
            bg=BLUE_DARK,
            fg=WHITE,
            font=(FONT, 9, "bold"),
            padx=12,
            pady=8,
        ).pack(side="right", padx=20)

    def _build_status(self):
        footer = tk.Frame(self.root, bg="#E8EDF1")
        footer.pack(fill="x", side="bottom")

        tk.Label(
            footer,
            textvariable=self.speed_text,
            bg="#E8EDF1",
            fg=TEXT,
            font=(FONT, 9, "bold"),
        ).pack(side="left", padx=(18, 12), pady=10)

        tk.Label(
            footer,
            textvariable=self.steering_text,
            bg="#E8EDF1",
            fg=MUTED,
            font=(FONT, 9, "bold"),
        ).pack(side="left", pady=10)

        # Update the displayed values from the widget's current command.
        self._update_status()

    def _update_status(self):
        if self.closing:
            return

        speed, steering = self.widget.get_command()
        self.speed_text.set(f"Speed: {speed:+.2f} m/s")
        self.steering_text.set(f"Steering: {steering:+.2f} rad")

        self.root.after(50, self._update_status)

    def close(self):
        if self.closing:
            return

        self.closing = True

        # Publish a final stop command before shutting down.
        try:
            self.node.publish_command(0.0, 0.0)
        except Exception as exc:
            self.node.get_logger().warning(
                f"Could not publish final stop command: {exc}"
            )

        self.widget.shutdown()
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)

    node = LimoTeleopNode()
    root = tk.Tk()
    app = LimoTeleopSimulation(root, node)

    try:
        root.mainloop()
    finally:
        app.widget.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
