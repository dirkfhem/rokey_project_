import rclpy  # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node  # Node 클래스
from custom_interfaces.srv import CustomServiceMessage

# 서비스 서버 노드 정의
class CustomServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')  # 노드 이름 설정
        # 'add_two_ints'라는 이름으로 서비스를 생성하고, 요청이 오면 add_callback을 호출
        self.brand
        self.count
        self.srv = self.create_service(CustomServiceMessage, 'add_two_ints', self.add_callback)

    # 서비스 요청이 들어오면 호출되는 콜백 함수
    def add_callback(self, request, response):
        # 요청받은 두 정수 출력 (로그)
        self.get_logger().info(f'Received request: {request.brand},{request.count}')
        '{"brands": brands, "counts": counts}'
        # 요청에 담긴 a, b를 더해서 응답 객체에 저장
        self.brand=request.brand
        self.count=request.count
        response = True
        # 응답 반환
        return response

# 메인 함수 — 노드 초기화 및 실행
def main(args=None):
    rclpy.init(args=args)  # rclpy 초기화
    node = CustomServer()  # 서비스 서버 노드 객체 생성
    rclpy.spin(node)  # 노드 실행 (콜백 기다림)
    node.destroy_node()  # 노드 제거
    rclpy.shutdown()  # rclpy 종료


if __name__ == "__main__":
    main()