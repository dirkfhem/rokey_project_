import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import tty
import termios
import json
from copy import deepcopy

from custom_pkg_srv_test.srv import CustomService


class StatusPublisher(Node):
    def __init__(self):
        super().__init__('status_publisher')
        # 상태 변수
        self.status = False  # True: moving, False: stopped
        self.publishing_enabled = True
        self.orders = None  # 수신된 주문 정보 저장
        self.num_orders = None
        self.key_sequence = []  # qwer 종료 시퀀스 추적
        # 토픽 퍼블리셔
        self.status_pub = self.create_publisher(String, '/status', 10)
        # 토픽 구독자
        # 서비스 생성
        super().__init__('service_server')
        self.srv = self.create_service(CustomService, 'custom_service', self.service_callback)
        # 타이머 설정 (1초마다 상태 발행)
        self.timer = self.create_timer(10.0, self.publish_status)
        # 키보드 입력 설정
        self.stdin_fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.stdin_fd)
        tty.setcbreak(self.stdin_fd)
        self.get_logger().info('Status publisher node started, publishing to /status topic')

    def publish_status(self):
        """상태 토픽 발행"""
        if not self.publishing_enabled:
            return
        msg = String()
        msg.data = 'moving' if self.status else 'stopped'
        self.status_pub.publish(msg)
        self.get_logger().info(f'Publishing status: {msg.data}')

    def handle_topic_able(self,flag):
        if(flag.isinstance()):
            self.publishing_enabled = flag
        else:
            print("boolean error : handle topic")

    def handle_status_robot(self,flag):
        if(flag.isinstance()):
            self.status = flag
        else:
            print("boolean error : robot status")
            
    def get_order(self):
        order = deepcopy(self.orders)
        num = deepcopy(self.num_orders)
        self.orders = None
        self.num_orders = None
        print(f"test : order = {order},{self.orders}")
        print(f"test : num = {num},{self.num_orders}")
        return order,num

    def service_callback(self, request, response):
        self.orders = request.brand  # 수신된 주문 정보 저장
        self.num_orders = request.count
        for num in range(len(request.brand)):
            self.get_logger().info(f'수신: brand={request.brand[num]}, count={request.count[num]}')
        response.success = True
        return response

    def cleanup(self):
        """터미널 설정 복원"""
        termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = StatusPublisher()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()