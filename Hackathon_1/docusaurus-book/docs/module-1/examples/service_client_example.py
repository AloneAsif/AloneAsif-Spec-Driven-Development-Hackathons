# Python Service Client Example
# This example demonstrates how to create a service client in Python using rclpy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from example_interfaces.srv import AddTwoInts
from std_msgs.msg import String
import time

class ServiceClientExample(Node):
    def __init__(self):
        super().__init__('service_client_example')

        # Create a service client
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Create a publisher for logging results
        self.result_publisher = self.create_publisher(
            String,
            'service_results',
            QoSProfile(depth=10)
        )

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.get_logger().info('Service client initialized and connected to service')

        # Variables to store the request and response
        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        """Send a request to the service and return the future"""
        self.request.a = a
        self.request.b = b

        self.get_logger().info(f'Sending request: {a} + {b}')
        future = self.cli.call_async(self.request)
        return future

    def send_request_with_callback(self, a, b):
        """Send a request to the service and set up a callback for the response"""
        self.request.a = a
        self.request.b = b

        self.get_logger().info(f'Sending request with callback: {a} + {b}')
        future = self.cli.call_async(self.request)
        future.add_done_callback(self.response_callback)
        return future

    def response_callback(self, future):
        """Callback function to handle the service response"""
        try:
            response = future.result()
            result_msg = String()
            result_msg.data = f'Service result: {response.sum}'
            self.result_publisher.publish(result_msg)
            self.get_logger().info(f'Service response received: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    client = ServiceClientExample()

    # Example 1: Synchronous service call (blocks until response received)
    try:
        # Send a request synchronously
        client.request.a = 10
        client.request.b = 20

        client.get_logger().info('Sending synchronous request...')
        response = client.cli.call(client.request)
        client.get_logger().info(f'Synchronous response: {response.sum}')
    except Exception as e:
        client.get_logger().error(f'Synchronous call failed: {e}')

    # Example 2: Asynchronous service calls
    # Send multiple requests asynchronously
    futures = []
    requests = [(1, 2), (3, 4), (5, 6), (7, 8), (9, 10)]

    for a, b in requests:
        future = client.send_request_with_callback(a, b)
        futures.append(future)

    # Spin to process callbacks
    try:
        # We'll spin for a limited time to allow callbacks to execute
        start_time = time.time()
        while time.time() - start_time < 5.0:  # Run for 5 seconds
            rclpy.spin_once(client, timeout_sec=0.1)
    except KeyboardInterrupt:
        client.get_logger().info('Client interrupted by user')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()