#!/usr/bin/env python3

import subprocess

from pynput import keyboard

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class RecordingNode(Node):
    """ROS 2 node for controlling recording from the keyboard."""

    def __init__(self):

        super().__init__('recording')

        self.recording = False

        # ========================================================
        # ROS 2 service clients
        # ========================================================

        self.start_client = self.create_client(
            Trigger,
            '/start_recording'
        )

        self.stop_client = self.create_client(
            Trigger,
            '/stop_recording'
        )

        # ========================================================
        # Keyboard listener
        # ========================================================

        # pynput listens to keyboard events in a separate thread.
        self.listener = keyboard.Listener(
            on_press=self.on_key_press
        )

        self.listener.start()

        self.get_logger().info(
            'Recording node ready. '
            'Press R to start/stop recording.'
        )

    # ============================================================
    # Keyboard
    # ============================================================

    def on_key_press(self, key):
        """Handle keyboard events."""

        try:

            if key.char.lower() != 'r':
                return

        except AttributeError:

            # Some keys, such as Shift or arrow keys, do not have
            # a character representation.
            return

        if self.recording:

            self.stop_recording()

        else:

            self.start_recording()

    # ============================================================
    # Start recording
    # ============================================================

    def start_recording(self):
        """Request the recorder node to start recording."""

        if not self.start_client.wait_for_service(
            timeout_sec=0.1
        ):

            self.get_logger().error(
                'Recording service is not available.'
            )

            return

        # call_async() sends the request without blocking the node.
        future = self.start_client.call_async(
            Trigger.Request()
        )

        # The callback is called when the service response is ready.
        future.add_done_callback(
            self.start_recording_callback
        )

    def start_recording_callback(self, future):
        """Handle the response from the start recording service."""

        try:

            response = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Failed to start recording: {error}'
            )

            return

        if response.success:

            self.recording = True

            self.get_logger().info(
                f'Recording started: {response.message}'
            )

        else:

            self.get_logger().error(
                response.message
            )

    # ============================================================
    # Stop recording
    # ============================================================

    def stop_recording(self):
        """Request the recorder node to stop recording."""

        if not self.stop_client.wait_for_service(
            timeout_sec=0.1
        ):

            self.get_logger().error(
                'Recording service is not available.'
            )

            return

        future = self.stop_client.call_async(
            Trigger.Request()
        )

        future.add_done_callback(
            self.stop_recording_callback
        )

    def stop_recording_callback(self, future):
        """Handle the response from the stop recording service."""

        try:

            response = future.result()

        except Exception as error:

            self.get_logger().error(
                f'Failed to stop recording: {error}'
            )

            return

        if response.success:

            self.recording = False

            self.get_logger().info(
                response.message
            )

            # The recorder returns the path of the recorded bag
            # in the service response.
            bag_path = response.message.replace(
                'Recording stopped: ',
                '',
                1
            )

            # Start the odometry results script with the recorded bag.
            subprocess.Popen([
                'ros2',
                'run',
                'room207_limo',
                'plot_odometry',
                bag_path,
            ])

        else:

            self.get_logger().error(
                response.message
            )

    # ============================================================
    # Shutdown
    # ============================================================

    def destroy_node(self):
        """Stop the keyboard listener and active recording."""

        self.listener.stop()

        if self.recording:

            self.get_logger().info(
                'Stopping recording before shutdown...'
            )

            if self.stop_client.wait_for_service(
                timeout_sec=1.0
            ):

                future = self.stop_client.call_async(
                    Trigger.Request()
                )

                rclpy.spin_until_future_complete(
                    self,
                    future,
                    timeout_sec=2.0
                )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = RecordingNode()

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