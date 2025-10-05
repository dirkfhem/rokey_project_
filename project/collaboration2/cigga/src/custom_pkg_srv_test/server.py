#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_pkg_srv_test.srv import CustomService
import time

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(CustomService, 'custom_service', self.service_callback)

    def service_callback(self, request, response):
        for num in range(len(request.brand)):
            self.get_logger().info(f'수신: brand={request.brand[num]}, count={request.count[num]}')
        response.success = True
        time.sleep(60)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()