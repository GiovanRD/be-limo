import signal
import subprocess
from datetime import datetime
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from std_srvs.srv import Trigger


class Recorder(Node):
    """ROS 2 node for recording selected topics with rosbag2."""

    def __init__(self):

        super().__init__('recorder')

        # ========================================================
        # Parameters
        # ========================================================

        workspace_dir = Path(
            get_package_prefix('recording')
        ).parent.parent

        self.declare_parameter('topics', [])

        self.declare_parameter('output_dir', '')

        output_dir = self.get_parameter('output_dir').value
        if not output_dir:
            output_dir = str(workspace_dir / 'data')

        self.topics = self.get_parameter('topics').value
        self.output_dir = Path(output_dir).expanduser()

        # ========================================================
        # Recorder state
        # ========================================================

        self.process = None

        # ========================================================
        # Services
        # ========================================================

        self.create_service(
            Trigger,
            '/start_recording',
            self.start_recording_callback
        )

        self.create_service(
            Trigger,
            '/stop_recording',
            self.stop_recording_callback
        )

        self.get_logger().info('Recorder ready.')

    # ============================================================
    # Start recording
    # ============================================================

    def start_recording_callback(self, request, response):
        """Start a rosbag2 recording."""

        if self._is_recording():

            response.success = False
            response.message = 'Recording is already active.'

            return response

        self.process = None

        if not self.topics:

            response.success = False
            response.message = 'No topics configured.'

            return response

        self.output_dir.mkdir(
            parents=True,
            exist_ok=True
        )

        bag_path = self._create_bag_path()

        command = [
            'ros2',
            'bag',
            'record',
            '-o',
            str(bag_path),
            *self.topics,
        ]

        self.get_logger().info(
            f'Starting recording: {bag_path}'
        )

        self.get_logger().info(
            f'Topics: {", ".join(self.topics)}'
        )

        try:

            self.process = subprocess.Popen(command)

        except OSError as error:

            self.process = None

            response.success = False
            response.message = (
                f'Failed to start rosbag2: {error}'
            )

            return response

        response.success = True
        response.message = str(bag_path)

        return response

    # ============================================================
    # Stop recording
    # ============================================================

    def stop_recording_callback(self, request, response):
        """Stop the active rosbag2 recording."""

        if not self._is_recording():

            self.process = None

            response.success = False
            response.message = 'Recording is not active.'

            return response

        self.get_logger().info('Stopping recording...')

        bag_path = self.process.args[
            self.process.args.index('-o') + 1
        ]

        self._stop_recording()

        response.success = True
        response.message = (
            f'Recording stopped: {bag_path}'
        )

        return response

    # ============================================================
    # Helpers
    # ============================================================

    def _is_recording(self):
        """Return True if the rosbag2 process is still running."""

        return (
            self.process is not None
            and self.process.poll() is None
        )

    def _create_bag_path(self):
        """Create a timestamped path for the next rosbag2."""

        timestamp = datetime.now().strftime(
            '%Y-%m-%d_%H-%M-%S'
        )

        return self.output_dir / f'rosbag_{timestamp}'

    def _stop_recording(self):
        """Stop the rosbag2 process gracefully if possible."""

        self.process.send_signal(signal.SIGINT)

        try:

            self.process.wait(timeout=5.0)

        except subprocess.TimeoutExpired:

            self.get_logger().warning(
                'rosbag2 did not stop gracefully. '
                'Forcing termination.'
            )

            self.process.kill()
            self.process.wait()

        self.process = None

    # ============================================================
    # Shutdown
    # ============================================================

    def destroy_node(self):

        if self._is_recording():

            self.get_logger().info(
                'Stopping recording before shutdown...'
            )

            self._stop_recording()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = Recorder()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()