#!/usr/bin/env python3

"""Integrated camera, teleoperation and recording GUI for Room 207."""

import math
import subprocess
import tkinter as tk
import time

import cv2
import numpy as np
import rclpy

from PIL import Image, ImageTk
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from std_srvs.srv import Trigger
from collections import deque


from nav_msgs.msg import Odometry
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from teleoperation.teleop_widget import TeleopWidget


# ============================================================
# GUI style
# ============================================================

BLUE = "#006699"
BLUE_DARK = "#004466"
BG = "#F3F6F8"
WHITE = "#FFFFFF"
TEXT = "#202B36"
MUTED = "#74818D"
RED = "#A62626"
GREEN = "#26734D"
BORDER = "#DFE6EB"
FOOTER_BG = "#E8EDF1"
FONT = "DejaVu Sans"


# ============================================================
# Teleoperation and camera parameters
# ============================================================

MAX_SPEED = 0.5
MAX_STEERING = math.pi / 6
PERIOD_MS = 20

CAMERA_TOPIC = "/rgb/image_raw"
CAMERA_WIDTH = 560
CAMERA_HEIGHT = 400
CAMERA_REFRESH_MS = 50

ODOM_TOPIC = "/odom"
PLOT_REFRESH_MS = 200
PLOT_WINDOW_SECONDS = 30.0


class Room207Node(Node):
    """ROS 2 interface for camera, teleoperation and recording."""

    def __init__(self):
        super().__init__("room207_gui")

        # Velocity commands. Remapped by demo.launch.py.
        self.cmd_pub = self.create_publisher(
            Twist, "/cmd_vel", 10
        )

        # Camera subscription.
        self.latest_image = None
        self.camera_sub = self.create_subscription(
            ImageMsg,
            CAMERA_TOPIC,
            self._camera_callback,
            10,
        )

        # Odometry subscription for live plots.
        self.odom_sub = self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self._odom_callback,
            20,
        )

        self.odom_start_time = None
        self.trajectory_x = deque(maxlen=20000)
        self.trajectory_y = deque(maxlen=20000)
        self.plot_time = deque(maxlen=10000)
        self.linear_speed = deque(maxlen=10000)
        self.angular_speed = deque(maxlen=10000)
        self.yaw_angle = deque(maxlen=10000)

        # Recording services.
        self.start_client = self.create_client(
            Trigger, "/start_recording"
        )
        self.stop_client = self.create_client(
            Trigger, "/stop_recording"
        )

        self.recording = False
        self.request_pending = False
        self.pending_action = None
        self.on_recording_changed = None

        self.get_logger().info(
            f"Room 207 GUI ready; camera topic: {CAMERA_TOPIC}"
        )

    # --------------------------------------------------------
    # Camera
    # --------------------------------------------------------

    def _camera_callback(self, msg):
        """Convert a ROS Image message to an RGB NumPy array."""

        try:
            if msg.encoding not in ("rgb8", "bgr8", "rgba8", "bgra8"):
                self.get_logger().warning(
                    f"Unsupported camera encoding: {msg.encoding}"
                )
                return

            channels = 4 if "a" in msg.encoding else 3
            row_size = msg.width * channels

            # Account for possible row padding in the ROS message.
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            raw = raw.reshape(msg.height, msg.step)
            raw = raw[:, :row_size]
            image = raw.reshape(
                msg.height, msg.width, channels
            )

            if msg.encoding == "rgb8":
                rgb = image
            elif msg.encoding == "bgr8":
                rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            elif msg.encoding == "rgba8":
                rgb = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
            else:
                rgb = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)

            # Store an independent copy for the Tkinter thread.
            self.latest_image = rgb.copy()

        except (ValueError, cv2.error) as exc:
            self.get_logger().warning(
                f"Could not decode camera image: {exc}"
            )

    # -------------------------------------------------------
    # Odometry
    # -------------------------------------------------------

    def _odom_callback(self, msg):
        """Store odometry data for the live plots."""

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        twist = msg.twist.twist

        # Convert the orientation quaternion to yaw.
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y**2 + orientation.z**2
        )
        yaw = math.atan2(sin_yaw, cos_yaw)

        # Elapsed time since the first received odometry message.
        now = time.monotonic()
        if self.odom_start_time is None:
            self.odom_start_time = now

        elapsed = now - self.odom_start_time

        self.trajectory_x.append(position.x)
        self.trajectory_y.append(position.y)
        self.plot_time.append(elapsed)
        self.linear_speed.append(twist.linear.x)
        self.angular_speed.append(twist.angular.z)
        self.yaw_angle.append(yaw)
    
    
    # --------------------------------------------------------
    # Teleoperation
    # --------------------------------------------------------

    def publish_command(self, speed, steering):
        """Publish a velocity and steering command."""

        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(steering)

        # Preserve the steering convention of the existing widget.
        if speed < 0.0:
            msg.angular.z = -msg.angular.z

        self.cmd_pub.publish(msg)

    # --------------------------------------------------------
    # Recording
    # --------------------------------------------------------

    def toggle_recording(self):
        """Start or stop recording without concurrent requests."""

        if self.request_pending:
            return False

        action = "stop" if self.recording else "start"
        client = (
            self.stop_client if action == "stop"
            else self.start_client
        )

        if not client.service_is_ready():
            message = (
                f"Recording service /{action}_recording "
                "is not available."
            )
            self.get_logger().error(message)
            self._notify(message, "error")
            return False

        self.request_pending = True
        self.pending_action = action
        self._notify(
            "Starting recording..." if action == "start"
            else "Stopping recording...",
            "pending",
        )

        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda result: self._recording_callback(result, action)
        )
        return True

    def _recording_callback(self, future, action):
        """Process a recording service response."""

        self.request_pending = False
        self.pending_action = None

        try:
            response = future.result()
        except Exception as exc:
            message = f"{action.capitalize()} recording failed: {exc}"
            self.get_logger().error(message)
            self._notify(message, "error")
            return

        if not response.success:
            message = (
                response.message
                or f"{action.capitalize()} recording failed."
            )
            self.get_logger().error(message)
            self._notify(message, "error")
            return

        if action == "start":
            self.recording = True
            self.get_logger().info(
                f"Recording started: {response.message}"
            )
            self._notify("Recording in progress", "recording")
            return

        self.recording = False
        self.get_logger().info(response.message)
        self._notify(
            "Recording stopped. Generating plots...", "plotting"
        )

        prefix = "Recording stopped:"
        message = response.message.strip()

        if not message.startswith(prefix):
            warning = (
                "Recording stopped, but the response did not "
                "contain the expected rosbag path."
            )
            self.get_logger().warning(warning)
            self._notify(warning, "error")
            return

        bag_path = message[len(prefix):].strip()
        if not bag_path:
            warning = "Recording stopped, but no rosbag path was returned."
            self.get_logger().warning(warning)
            self._notify(warning, "error")
            return

        self.launch_plotting(bag_path)

    def _notify(self, message, state):
        """Notify the GUI, if its callback is registered."""

        if self.on_recording_changed is not None:
            self.on_recording_changed(message, state)

    # --------------------------------------------------------
    # Plotting
    # --------------------------------------------------------

    def launch_plotting(self, bag_path):
        """Launch the existing odometry plotting executable."""

        try:
            subprocess.Popen([
                "ros2", "run", "room207_limo",
                "plot_odometry", bag_path,
            ])
            self.get_logger().info(
                f"Odometry plotting launched for: {bag_path}"
            )
            self._notify(
                "Plots launched. PDFs are saved in data/.", "normal"
            )
        except OSError as exc:
            message = f"Could not launch odometry plotting: {exc}"
            self.get_logger().error(message)
            self._notify(message, "error")


class Room207GUI:
    """Combined camera, teleoperation and recording interface."""

    def __init__(self, root, node):
        self.root = root
        self.node = node

        self.closing = False
        self.destroyed = False
        self.shutdown_stop_attempted = False
        self.camera_photo = None

        root.title("ENSEM | Room 207 LIMO")
        root.geometry("1600x1000")
        root.minsize(1200, 760)
        root.configure(bg=BG)

        self.record_status = tk.StringVar(value="Not recording")
        self.record_button_text = tk.StringVar(
            value="Start recording  (R)"
        )
        self.speed_text = tk.StringVar(value="Speed: +0.00 m/s")
        self.steering_text = tk.StringVar(
            value="Steering: +0.00 rad"
        )
        self.camera_status = tk.StringVar(value="Waiting for camera...")

        self._build_header()
        self._build_main_area()
        self._build_footer()

        self.node.on_recording_changed = self._update_recording_status

        root.bind("<KeyPress-r>", self._on_r)
        root.bind("<KeyPress-R>", self._on_r)
        root.bind("<space>", self._on_space)
        root.protocol("WM_DELETE_WINDOW", self.close)

        root.after(100, self._poll_ros)
        root.after(CAMERA_REFRESH_MS, self._refresh_camera)
        root.after(PLOT_REFRESH_MS, self._refresh_plots)
        root.after(150, root.focus_force)

    # --------------------------------------------------------
    # Header
    # --------------------------------------------------------

    def _build_header(self):
        header = tk.Frame(self.root, bg=BLUE, height=76)
        header.pack(fill="x")
        header.pack_propagate(False)

        left = tk.Frame(header, bg=BLUE)
        left.pack(side="left", padx=22, pady=12)

        tk.Label(
            left, text="ENSEM", bg=BLUE, fg=WHITE,
            font=(FONT, 10, "bold"),
        ).pack(anchor="w")

        tk.Label(
            left, text="ROOM 207 | LIMO CONTROL",
            bg=BLUE, fg=WHITE, font=(FONT, 16, "bold"),
        ).pack(anchor="w", pady=(3, 0))

        tk.Label(
            header, text="ROS 2", bg=BLUE_DARK, fg=WHITE,
            font=(FONT, 9, "bold"), padx=12, pady=8,
        ).pack(side="right", padx=20)

    # --------------------------------------------------------
    # Main area
    # --------------------------------------------------------
        
    def _build_main_area(self):
        main = tk.Frame(self.root, bg=BG)
        main.pack(fill="both", expand=True, padx=12, pady=10)

        # Two columns with fixed proportions.
        main.grid_columnconfigure(0, weight=0, minsize=800)
        main.grid_columnconfigure(1, weight=1, minsize=450)
        main.grid_rowconfigure(0, weight=1)

        self.left_panel = tk.Frame(main, bg=BG, width=680)
        self.left_panel.pack_propagate(False)
        self.left_panel.grid(
            row=0, column=0, sticky="nsew", padx=(0, 8)
        )
        self.left_panel.grid_propagate(False)

        self.right_panel = tk.Frame(main, bg=BG, width=450)
        self.right_panel.grid(
            row=0, column=1, sticky="nsew", padx=(8, 0)
        )
        self.right_panel.grid_propagate(False)

        # Left column.
        self._build_camera()
        self._build_recording()

        self.teleop = TeleopWidget(
            self.left_panel,
            on_command=self._publish_command,
            on_close=self.close,
            max_speed=MAX_SPEED,
            max_steering=MAX_STEERING,
            period_ms=PERIOD_MS,
        )
        self.teleop.pack(
            fill="both",
            expand=True,
            padx=12,
            pady=10,
        )
        self.teleop.pack_propagate(False)

        self.root.update_idletasks()

        print("\n--- Géométrie des conteneurs ---")
        for name, widget in [
            ("root", self.root),
            ("main", self.left_panel.master),
            ("left_panel", self.left_panel),
            ("right_panel", self.right_panel),
            ("teleop", self.teleop),
        ]:
            print(
                name,
                "size =", widget.winfo_width(), "x", widget.winfo_height(),
                "requested =", widget.winfo_reqwidth(), "x", widget.winfo_reqheight(),
                "manager =", widget.winfo_manager(),
            )

        print("Grid main :", self.left_panel.master.grid_size())
        print("Grid left :", self.left_panel.grid_info())
        print("Grid right :", self.right_panel.grid_info())

        # Right column.
        self._build_live_plots()

        self.root.after(1000, lambda: print(
            "left_panel :", self.left_panel.winfo_height(),
            "| camera :", self.camera_label.winfo_height(),
            "| teleop :", self.teleop.winfo_height()
        ))

    # --------------------------------------------------------
    # Camera display
    # --------------------------------------------------------

    
    def _build_camera(self):
        panel = tk.Frame(
            self.left_panel,
            bg=WHITE,
            height=CAMERA_HEIGHT + 55,
            highlightbackground=BORDER,
            highlightthickness=1,
        )
        panel.pack(fill="x", padx=24, pady=(12, 0))
        panel.pack_propagate(False)

        top = tk.Frame(panel, bg=WHITE)
        top.pack(fill="x", padx=12, pady=(8, 5))

        tk.Label(
            top,
            text="LIVE CAMERA",
            bg=WHITE,
            fg=TEXT,
            font=(FONT, 10, "bold"),
        ).pack(side="left")

        tk.Label(
            top,
            textvariable=self.camera_status,
            bg=WHITE,
            fg=MUTED,
            font=(FONT, 9),
        ).pack(side="right")

        self.camera_label = tk.Label(
            panel,
            text="Waiting for /rgb/image_raw...",
            bg="#17232D",
            fg=WHITE,
            font=(FONT, 10),
            compound="center",
            anchor="center",
        )
        self.camera_label.pack(
            fill="both",
            expand=True,
            padx=10,
            pady=(0, 10),
        )


    def _refresh_camera(self):
        """Display the latest camera image without blocking Tkinter."""

        if self.destroyed:
            return

        image = self.node.latest_image

        if image is not None:
            try:
                height, width = image.shape[:2]

                # Fit the image within the configured dimensions.
                scale = min(
                    CAMERA_WIDTH / width,
                    CAMERA_HEIGHT / height,
                )
                new_width = max(1, int(width * scale))
                new_height = max(1, int(height * scale))

                resized = cv2.resize(
                    image,
                    (new_width, new_height),
                    interpolation=cv2.INTER_AREA,
                )

                pil_image = Image.fromarray(resized)
                self.camera_photo = ImageTk.PhotoImage(pil_image)

                self.camera_label.configure(
                    image=self.camera_photo,
                    text="",
                )
                self.camera_status.set(f"{width} × {height}")

            except (ValueError, tk.TclError) as exc:
                self.node.get_logger().warning(
                    f"Camera display error: {exc}"
                )

        elif self.camera_status.get() != "Waiting for camera...":
            self.camera_status.set("Waiting for camera...")

        self.root.after(CAMERA_REFRESH_MS, self._refresh_camera)

    # --------------------------------------------------------
    # Recording controls
    # --------------------------------------------------------

    def _build_recording(self):
        panel = tk.Frame(
            self.left_panel, bg=WHITE,
            highlightbackground=BORDER, highlightthickness=1,
        )
        panel.pack(fill="x", padx=24, pady=(12, 0))

        top = tk.Frame(panel, bg=WHITE)
        top.pack(fill="x", padx=16, pady=(10, 5))

        tk.Label(
            top, text="RECORDING", bg=WHITE, fg=TEXT,
            font=(FONT, 11, "bold"),
        ).pack(side="left")

        self.status_label = tk.Label(
            top, textvariable=self.record_status,
            bg=WHITE, fg=MUTED, font=(FONT, 9, "bold"),
        )
        self.status_label.pack(side="right")

        self.record_button = tk.Button(
            panel,
            textvariable=self.record_button_text,
            command=self._toggle_recording,
            bg=RED, fg=WHITE,
            activebackground="#861F1F",
            activeforeground=WHITE,
            font=(FONT, 10, "bold"),
            relief="flat", bd=0, cursor="hand2",
            padx=12, pady=8,
        )
        self.record_button.pack(
            fill="x", padx=16, pady=(4, 12)
        )

    # --------------------------------------------------------
    # Live plots
    # --------------------------------------------------------
    
    
    
    
    
    def _build_live_plots(self):
        panel = tk.Frame(self.right_panel, bg=WHITE)
        panel.pack(fill="both", expand=True)

        self.odom_status = tk.StringVar(value="Waiting for /odom...")

        # Physical dimensions of the circuit map, in metres.
        MAP_WIDTH = 4.88
        MAP_HEIGHT = 6.20

        half_width = MAP_WIDTH / 2.0
        half_height = MAP_HEIGHT / 2.0

        self.fig = Figure(figsize=(5, 8), dpi=100)
        self.fig.subplots_adjust(
            left=0.16,
            right=0.96,
            top=0.92,
            bottom=0.08,
        )

        self.ax_trajectory = self.fig.add_subplot(111)
        self.ax_trajectory.set_title("LIMO trajectory")
        self.ax_trajectory.set_xlabel("x (m)")
        self.ax_trajectory.set_ylabel("y (m)")
        self.ax_trajectory.set_xlim(-half_width, half_width)
        self.ax_trajectory.set_ylim(-half_height, half_height)
        self.ax_trajectory.set_aspect("equal", adjustable="box")
        self.ax_trajectory.grid(False)

        try:
            map_file = (
                Path(get_package_share_directory("qcar_large_track"))
                / "models"
                / "qcar_large_track"
                / "materials"
                / "textures"
                / "SDCS_MapLayout.png"
            )

            # Load the image without rotating or resizing it.
            map_image = np.asarray(
                Image.open(map_file).convert("RGB")
            )

            self.ax_trajectory.imshow(
                map_image,
                extent=[
                    -half_width,
                    half_width,
                    -half_height,
                    half_height,
                ],
                origin="upper",
                aspect="equal",
            )

        except (OSError, LookupError, ValueError) as exc:
            self.ax_trajectory.set_facecolor("#E8EDF1")
            print(f"Warning: circuit map unavailable: {exc}")

        self.trajectory_line, = self.ax_trajectory.plot(
            [],
            [],
            color="#E53935",
            linewidth=1.8,
            label="Trajectory",
        )
        self.ax_trajectory.legend(loc="upper right")

        self.plot_canvas = FigureCanvasTkAgg(
            self.fig,
            master=panel,
        )
        self.plot_canvas.get_tk_widget().pack(
            side="top",
            fill="both",
            expand=True,
        )

        tk.Label(
            panel,
            textvariable=self.odom_status,
            bg=WHITE,
            fg=MUTED,
            font=(FONT, 9),
        ).pack(side="bottom", fill="x", padx=8, pady=4)

        self.plot_canvas.draw()



    def _refresh_plots(self):
        if self.node.plot_time:
            x = np.asarray(self.node.trajectory_x)
            y = np.asarray(self.node.trajectory_y)

            self.trajectory_line.set_data(x, y)

            self.odom_status.set(
                f"Receiving /odom ({len(x)} samples)"
            )
        else:
            self.odom_status.set("Waiting for /odom...")

        self.plot_canvas.draw_idle()
        self.root.after(
            PLOT_REFRESH_MS,
            self._refresh_plots,
        )




    # --------------------------------------------------------
    # Footer
    # --------------------------------------------------------

    def _build_footer(self):
        footer = tk.Frame(self.root, bg=FOOTER_BG)
        footer.pack(fill="x", side="bottom")

        tk.Label(
            footer, textvariable=self.speed_text,
            bg=FOOTER_BG, fg=TEXT, font=(FONT, 9, "bold"),
        ).pack(side="left", padx=(18, 12), pady=9)

        tk.Label(
            footer, textvariable=self.steering_text,
            bg=FOOTER_BG, fg=MUTED, font=(FONT, 9, "bold"),
        ).pack(side="left", pady=9)

    # --------------------------------------------------------
    # GUI callbacks
    # --------------------------------------------------------

    def _publish_command(self, speed, steering):
        if self.closing:
            return

        self.node.publish_command(speed, steering)
        self.speed_text.set(f"Speed: {speed:+.2f} m/s")
        self.steering_text.set(f"Steering: {steering:+.2f} rad")

    def _toggle_recording(self):
        if self.closing or self.node.request_pending:
            return
        self.node.toggle_recording()

    def _on_r(self, _event=None):
        self._toggle_recording()
        return "break"

    def _on_space(self, _event=None):
        """Stop the robot without stopping the recording."""
        if not self.closing:
            self._publish_command(0.0, 0.0)
        return "break"

    def _update_recording_status(self, message, state):
        if self.destroyed:
            return

        self.record_status.set(message)

        if state == "recording":
            self.record_button_text.set("Stop recording  (R)")
            self.record_button.configure(bg=BLUE_DARK, state="normal")
            self.status_label.configure(fg=RED)
        elif state == "pending":
            self.record_button.configure(state="disabled")
            self.status_label.configure(fg=MUTED)
        elif state == "plotting":
            self.record_button_text.set("Start recording  (R)")
            self.record_button.configure(bg=RED, state="normal")
            self.status_label.configure(fg=BLUE)
        elif state == "error":
            self.record_button_text.set("Start recording  (R)")
            self.record_button.configure(bg=RED, state="normal")
            self.status_label.configure(fg=RED)
        else:
            self.record_button_text.set("Start recording  (R)")
            self.record_button.configure(bg=RED, state="normal")
            self.status_label.configure(fg=GREEN)

    # --------------------------------------------------------
    # ROS event loop and shutdown
    # --------------------------------------------------------

    def _poll_ros(self):
        if self.destroyed:
            return

        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as exc:
            self.node.get_logger().error(
                f"ROS callback error: {exc}"
            )

        if self.closing:
            self._handle_shutdown()
        else:
            self._sync_recording_controls()

        if not self.destroyed:
            self.root.after(20, self._poll_ros)

    def _sync_recording_controls(self):
        if self.node.request_pending:
            self.record_button.configure(state="disabled")
            return

        self.record_button.configure(state="normal")
        if self.node.recording:
            self.record_button_text.set("Stop recording  (R)")
            self.record_button.configure(bg=BLUE_DARK)
            self.status_label.configure(fg=RED)
        else:
            self.record_button_text.set("Start recording  (R)")
            self.record_button.configure(bg=RED)

    def close(self):
        """Stop the robot and finalize an active recording."""
        if self.closing or self.destroyed:
            return

        self.closing = True
        self.node.publish_command(0.0, 0.0)
        self.record_button.configure(state="disabled")
        self.record_status.set("Closing safely...")
        self.teleop.shutdown()
        self._handle_shutdown()

    def _handle_shutdown(self):
        if self.destroyed:
            return

        if self.node.request_pending:
            self.record_status.set(
                "Waiting for the recording service..."
            )
            return

        if (
            self.node.recording
            and not self.shutdown_stop_attempted
        ):
            self.shutdown_stop_attempted = True
            self.record_status.set(
                "Finalizing recording before closing..."
            )
            if self.node.toggle_recording():
                return

            self.record_status.set(
                "Could not stop recording. Closing GUI."
            )
            self._destroy()
            return

        self._destroy()

    def _destroy(self):
        if self.destroyed:
            return

        self.destroyed = True
        try:
            self.root.destroy()
        except tk.TclError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Room207Node()
    root = tk.Tk()

    try:
        Room207GUI(root, node)
        root.mainloop()
    finally:
        try:
            node.publish_command(0.0, 0.0)
        except Exception:
            pass

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()