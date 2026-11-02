import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class StartRecording(Node):

    def __init__(self):

        super().__init__('start_recording')

        self.client = self.create_client(
            Trigger,
            '/start_recording'
        )

    def start(self):

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
        # Start recording
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
                f'Recording started: {response.message}'
            )

            return True

        self.get_logger().error(
            response.message
        )

        return False


def main(args=None):

    rclpy.init(args=args)

    node = StartRecording()

    try:

        node.start()

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()