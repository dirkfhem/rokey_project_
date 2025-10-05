

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 240)
        self.declare_parameter('settings_file', 'src/cam_settings.yaml')
        
        # Get parameters
        self.camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        self.frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.camera_frame_id = self.get_parameter('camera_frame_id').get_parameter_value().string_value
        self.jpeg_quality = self.get_parameter('jpeg_quality').get_parameter_value().integer_value
        self.calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.settings_file = self.get_parameter('settings_file').get_parameter_value().string_value
        
        # Publishers
        self.image_pub = self.create_publisher(Image, f'/{self.camera_name}/image_raw', 10)
        self.compressed_pub = self.create_publisher(CompressedImage, f'/{self.camera_name}/image_raw/compressed', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, f'/{self.camera_name}/camera_info', 10)
        self.filtered_pub = self.create_publisher(Image, f'/{self.camera_name}/image_filtered', 10)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera setup
        self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            for alt_id in range(1, 50):
                if alt_id != self.camera_id:
                    self.cap = cv2.VideoCapture(alt_id, cv2.CAP_V4L2)
                    if self.cap.isOpened():
                        self.camera_id = alt_id
                        self.get_logger().info(f'Success: Connected to camera device {alt_id}')
                        break
            if not self.cap.isOpened():
                self.get_logger().error('Failed to open any camera device')
                return
                
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)  # Disable auto white balance
        
        # Get actual camera properties
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(f'Camera opened: {actual_width}x{actual_height} @ {actual_fps} FPS')
        
        # Load camera settings
        self.camera_settings = self.load_camera_settings()
        if self.camera_settings:
            self.apply_camera_settings(self.cap, self.camera_settings['camera'])
        
        # Load camera calibration
        self.camera_info_msg = self.load_camera_calibration()
        
        # JPEG compression parameters
        self.jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        
        # Timer for publishing
        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Camera node started')
        self.get_logger().info(f'Publishing topics:')
        self.get_logger().info(f'  - /{self.camera_name}/image_raw')
        self.get_logger().info(f'  - /{self.camera_name}/image_raw/compressed')
        self.get_logger().info(f'  - /{self.camera_name}/camera_info')
        self.get_logger().info(f'  - /{self.camera_name}/image_filtered')
    
    def load_camera_settings(self):
        """Load camera settings from YAML file"""
        try:
            if os.path.exists(self.settings_file):
                with open(self.settings_file, 'r') as file:
                    settings = yaml.safe_load(file)
                self.get_logger().info(f'Loaded camera settings from: {self.settings_file}')
                return settings
            else:
                self.get_logger().warn(f'Camera settings file not found: {self.settings_file}')
                return None
        except Exception as e:
            self.get_logger().error(f'Failed to load camera settings: {e}')
            return None
    
    def apply_camera_settings(self, cap, settings):
        """Apply camera settings: exposure, white balance, saturation"""
        try:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, settings['resolution']['width'])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, settings['resolution']['height'])
            for prop, value in [
                (cv2.CAP_PROP_SATURATION, settings['saturation']),
                (cv2.CAP_PROP_EXPOSURE, settings['exposure']),
                (cv2.CAP_PROP_WB_TEMPERATURE, settings['white_balance'])
            ]:
                if cap.get(prop) != -1:  # Check if property is supported
                    cap.set(prop, value)
                    if prop == cv2.CAP_PROP_WB_TEMPERATURE:
                        actual_value = cap.get(prop)
                        if abs(actual_value - value) > 1:
                            self.get_logger().warn(f'White balance set to {value} but actual value is {actual_value}')
        except Exception as e:
            self.get_logger().warn(f'Error applying camera settings: {e}')
    
    def load_camera_calibration(self):
        """Load camera calibration from file or use default values"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.camera_frame_id
        
        calibration_loaded = False
        
        if self.calibration_file:
            try:
                if os.path.exists(self.calibration_file):
                    with open(self.calibration_file, 'r') as file:
                        calib_data = yaml.safe_load(file)
                    camera_info = self.parse_calibration_yaml(calib_data)
                    calibration_loaded = True
                    self.get_logger().info(f'Loaded calibration from: {self.calibration_file}')
                else:
                    self.get_logger().warn(f'Calibration file not found: {self.calibration_file}')
            except Exception as e:
                self.get_logger().error(f'Failed to load calibration file: {e}')
        
        if not calibration_loaded:
            camera_info_dir = os.path.expanduser('~/.ros/camera_info')
            camera_info_file = os.path.join(camera_info_dir, f'{self.camera_name}.yaml')
            
            if os.path.exists(camera_info_file):
                try:
                    with open(camera_info_file, 'r') as file:
                        calib_data = yaml.safe_load(file)
                    camera_info = self.parse_calibration_yaml(calib_data)
                    calibration_loaded = True
                    self.get_logger().info(f'Loaded calibration from: {camera_info_file}')
                except Exception as e:
                    self.get_logger().error(f'Failed to load camera info file: {e}')
        
        if not calibration_loaded:
            camera_info = self.create_default_camera_info()
            self.get_logger().warn('Using default camera calibration parameters')
            self.get_logger().warn('Please calibrate your camera for accurate results')
        
        return camera_info
    
    def parse_calibration_yaml(self, calib_data):
        """Parse calibration data from YAML"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.camera_frame_id
        
        camera_info.width = calib_data.get('image_width', self.image_width)
        camera_info.height = calib_data.get('image_height', self.image_height)
        
        if 'camera_matrix' in calib_data:
            camera_info.k = calib_data['camera_matrix']['data']
        
        if 'distortion_coefficients' in calib_data:
            camera_info.d = calib_data['distortion_coefficients']['data']
        
        if 'rectification_matrix' in calib_data:
            camera_info.r = calib_data['rectification_matrix']['data']
        else:
            camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        if 'projection_matrix' in calib_data:
            camera_info.p = calib_data['projection_matrix']['data']
        
        camera_info.distortion_model = calib_data.get('distortion_model', 'plumb_bob')
        
        return camera_info
    
    def create_default_camera_info(self):
        """Create default camera info"""
        camera_info = CameraInfo()
        camera_info.header.frame_id = self.camera_frame_id
        camera_info.width = self.image_width
        camera_info.height = self.image_height
        
        fx = 525.0
        fy = 525.0
        cx = self.image_width / 2.0
        cy = self.image_height / 2.0
        
        camera_info.k = [fx, 0.0, cx,
                        0.0, fy, cy,
                        0.0, 0.0, 1.0]
        
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        camera_info.r = [1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0]
        
        camera_info.p = [fx, 0.0, cx, 0.0,
                        0.0, fy, cy, 0.0,
                        0.0, 0.0, 1.0, 0.0]
        
        camera_info.distortion_model = "plumb_bob"
        
        return camera_info
    
    def timer_callback(self):
        """Main callback to capture and publish images"""
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
        
        timestamp = self.get_clock().now().to_msg()
        
        # Publish raw image
        try:
            raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            raw_msg.header.stamp = timestamp
            raw_msg.header.frame_id = self.camera_frame_id
            self.image_pub.publish(raw_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing raw image: {e}')
        
        # Publish compressed image
        try:
            success, encoded_image = cv2.imencode('.jpg', frame, self.jpeg_params)
            if success:
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = timestamp
                compressed_msg.header.frame_id = self.camera_frame_id
                compressed_msg.format = "jpeg"
                compressed_msg.data = encoded_image.tobytes()
                self.compressed_pub.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing compressed image: {e}')
        
        # Publish camera info
        try:
            self.camera_info_msg.header.stamp = timestamp
            self.camera_info_pub.publish(self.camera_info_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing camera info: {e}')
        
        # Publish filtered image (same as raw, settings applied via hardware)
        try:
            filtered_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            filtered_msg.header.stamp = timestamp
            filtered_msg.header.frame_id = self.camera_frame_id
            self.filtered_pub.publish(filtered_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing filtered image: {e}')
    
    def destroy_node(self):
        """Clean up resources"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()