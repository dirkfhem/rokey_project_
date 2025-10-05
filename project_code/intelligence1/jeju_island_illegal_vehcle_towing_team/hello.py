import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import tf2_ros
import rclpy.duration
import threading

class tf_sever(Node):
    def __init__(self):
        super().__init__('yolo_depth_green_detector')
        # QoS 설정
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        # 토픽 구독
        self.subscription = self.create_subscription(
            String, 'pose_label', self.listener_callback, qos
        )
        # 마커 퍼블리셔cv2
        self.marker_pub = self.create_publisher(Marker, 'detected_objects_marker', qos)
        self.marker_id = 0
        # TF2 버퍼와 리스너
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # 초기화 변수
        self.camera_frame_id = "oakd_rgb_camera_frame"  # 카메라 프레임 ID (필요 시 수정)
        self.should_shutdown = False
        self.get_logger().info("Illegal vehicle subscriber started")

    def listener_callback(self, msg):
        try:
            # 메시지 파싱
            x_str, y_str, z_str, label = msg.data.split(',')
            x, y, z = float(x_str), float(y_str), float(z_str)
            self.get_logger().info(f"Received: x={x:.2f}, y={y:.2f}, z={z:.2f}, label={label}")
            # TF 변환: 카메라 프레임 -> 맵 프레임
            pt_camera = PointStamped()
            pt_camera.header.frame_id = self.camera_frame_id
            pt_camera.header.stamp = self.get_clock().now().to_msg()
            pt_camera.point.x = x
            pt_camera.point.y = y
            pt_camera.point.z = z
            try:
                pt_map = self.tf_buffer.transform(
                    pt_camera, 'map', timeout=rclpy.duration.Duration(seconds=0.5)
                )
                map_x = pt_map.point.x
                map_y = pt_map.point.y
                map_z = pt_map.point.z
                self.get_logger().info(f"[Map] x={map_x:.2f}m, y={map_y:.2f}m, z={map_z:.2f}m, label={label}")
                # RViz 마커 퍼블리시
                self.publish_marker(map_x, map_y, map_z, label)
            except Exception as e:
                self.get_logger().warn(f"TF 변환 실패: {e}")
        except ValueError:
            self.get_logger().warn(f"Invalid message format: {msg.data}")

    def publish_marker(self, x, y, z, label):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'detected_objects'
        marker.id = self.marker_id
        self.marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.25
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rclpy.duration.Duration(seconds=3).to_msg()
        marker.text = label
        self.marker_pub.publish(marker)
        self.get_logger().info(f"Published marker: {label} at ({x:.2f}, {y:.2f}, {z:.2f})")

def ros_spin_thread(node):
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()

def main():
    rclpy.init()
    node = tf_sever()
    ros_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    ros_thread.start()
    try:
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node)
            if 0xFF == ord('q'):
                node.should_shutdown = True
                node.get_logger().info("Q 눌러 종료합니다.")
                break
    except KeyboardInterrupt:
        node.get_logger().info("키보드 인터럽트로 종료합니다.")
    finally:
        rclpy.shutdown()
        ros_thread.join()

if __name__ == '__main__':
    main()