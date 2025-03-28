#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class RemoteController(Node):
    def __init__(self):
        super().__init__('remote_controller')
        self.client = self.create_client(Trigger, 'drone_command')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for server...')

        self.send_command()

    def send_command(self):
        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.command_response)

    def command_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"Response: {response.success}, Message: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    controller = RemoteController()
    rclpy.spin(controller)


if __name__ == '__main__':
    main()
