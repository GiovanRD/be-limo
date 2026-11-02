import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class StopRecording(Node):
    """ROS 2 node for requesting the stop of a rosbag2 recording."""

    def __init__(self):

        super().__init__('stop_recording')

        self.client = self.create_client(
            Trigger,
            '/stop_recording'
        )

    def stop(self):
        """Request the recorder node to stop the active recording."""

        # ========================================================
        # Wait for recorder
        # ========================================================

        if not self.client.wait_for_service(
            timeout_sec=2.0
        ):

            self.get_logger().error(
                'Recorder is not running.'
            )

            return False

        # ========================================================
        # Stop recording
        # ========================================================

        request = Trigger.Request()

        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        response = future.result()

        if response.success:

            self.get_logger().info(
                'Recording stopped.'
            )

            return True

        self.get_logger().error(
            response.message
        )

        return False


def main(args=None):

    rclpy.init(args=args)

    node = StopRecording()

    try:

        node.stop()

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()