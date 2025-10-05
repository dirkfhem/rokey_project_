#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_pkg_srv_test.srv import CustomService

class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(CustomService, 'custom_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스를 기다리는 중...')
        self.req = CustomService.Request()

    def send_request(self, brand, count):
        self.req.brand = brand
        self.req.count = count
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    client = ServiceClient()
    response = client.send_request('MyBrand', 5)
    client.get_logger().info(f'결과: success={response.success}')
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()