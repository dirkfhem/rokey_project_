import cv2
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

    def publish_image(self, cv_image):
        try:
            msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish image: {e}")

def load_camera_settings(yaml_file):
    try:
        with open(yaml_file, 'r') as file:
            return yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Error: {yaml_file} not found.")
        return None

def save_camera_settings(yaml_file, config):
    try:
        with open(yaml_file, 'w') as file:
            yaml.safe_dump(config, file, default_flow_style=False)
        print("Settings saved to cam_settings.yaml")
    except Exception as e:
        print(f"Error saving settings: {e}")

def apply_camera_settings(cap, settings):
    try:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, settings['resolution']['width'])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, settings['resolution']['height'])
        for prop, value in [
            (cv2.CAP_PROP_BRIGHTNESS, settings['brightness']),
            (cv2.CAP_PROP_CONTRAST, settings['contrast']),
            (cv2.CAP_PROP_SATURATION, settings['saturation']),
            (cv2.CAP_PROP_HUE, settings['hue']),
            (cv2.CAP_PROP_EXPOSURE, settings['exposure']),
            (cv2.CAP_PROP_WB_TEMPERATURE, settings['white_balance'])
        ]:
            if cap.get(prop) != -1:  # xCheck if property is supported
                cap.set(prop, value)
                # Verify if white balance was set correctly
                if prop == cv2.CAP_PROP_WB_TEMPERATURE:
                    actual_value = cap.get(prop)
                    if abs(actual_value - value) > 1:
                        print(f"Warning: White balance set to {value} but actual value is {actual_value}")
    except Exception as e:
        print(f"Warning: Error applying camera settings: {e}")

def create_color_mask(hsv, color_range):
    lower = np.array([color_range['h_min'], color_range['s_min'], color_range['v_min']])
    upper = np.array([color_range['h_max'], color_range['s_max'], color_range['v_max']])
    return cv2.inRange(hsv, lower, upper)

def apply_bright_filter(image, threshold):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return cv2.bitwise_not(cv2.inRange(gray, threshold, 255))

def create_trackbars(config):
    cv2.namedWindow('Trackbars', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Trackbars', 400, 400)
    for color, ranges in config['color_ranges'].items():
        for key, value in ranges.items():
            max_val = 180 if key.startswith('h_') else 255
            cv2.createTrackbar(f"{color} {key}", 'Trackbars', value, max_val, lambda x: None)
    cv2.createTrackbar('White Balance', 'Trackbars', config['camera']['white_balance'], 10000, lambda x: None)
    cv2.createTrackbar('Yellow Bright Filter', 'Trackbars', int(config['preprocessing']['yellow_bright_filter']['apply']), 1, lambda x: None)
    cv2.createTrackbar('Yellow Bright Threshold', 'Trackbars', config['preprocessing']['yellow_bright_filter']['threshold'], 255, lambda x: None)
    cv2.createTrackbar('White Bright Filter', 'Trackbars', int(config['preprocessing']['white_bright_filter']['apply']), 1, lambda x: None)
    cv2.createTrackbar('White Bright Threshold', 'Trackbars', config['preprocessing']['white_bright_filter']['threshold'], 255, lambda x: None)

def main():
    rclpy.init()
    node = CameraPublisher()
    
    # Load configuration
    config = load_camera_settings('src/cam_settings.yaml')
    if not config:
        rclpy.shutdown()
        return

    # Initialize camera
    cap = cv2.VideoCapture(config['camera']['device_id'], cv2.CAP_V4L2)
    if not cap.isOpened():
        for alt_id in range(1,50):
            if alt_id != config['camera']['device_id']:
                cap = cv2.VideoCapture(alt_id, cv2.CAP_V4L2)
                if cap.isOpened():
                    config['camera']['device_id'] = alt_id
                    print(f"Success: Connected to camera device {alt_id}")
                    break
        if not cap.isOpened():
            print("Error: No camera device available.")
            rclpy.shutdown()
            return

    # Ensure auto white balance is disabled
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    apply_camera_settings(cap, config['camera'])
    
    trackbars_visible = False
    display_mode = 'original'
    display_window_exists = False
    
    while rclpy.ok():
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to read frame.")
            break

        # Update settings from trackbars if visible
        if trackbars_visible:
            config['camera']['white_balance'] = cv2.getTrackbarPos('White Balance', 'Trackbars')
            config['preprocessing']['yellow_bright_filter']['apply'] = cv2.getTrackbarPos('Yellow Bright Filter', 'Trackbars') == 1
            config['preprocessing']['yellow_bright_filter']['threshold'] = cv2.getTrackbarPos('Yellow Bright Threshold', 'Trackbars')
            config['preprocessing']['white_bright_filter']['apply'] = cv2.getTrackbarPos('White Bright Filter', 'Trackbars') == 1
            config['preprocessing']['white_bright_filter']['threshold'] = cv2.getTrackbarPos('White Bright Threshold', 'Trackbars')
            for color in ['yellow', 'white']:
                for key in config['color_ranges'][color]:
                    config['color_ranges'][color][key] = cv2.getTrackbarPos(f"{color} {key}", 'Trackbars')
            apply_camera_settings(cap, config['camera'])

        # Preprocessing
        processed_frame = frame
        yellow_bright_mask = None
        white_bright_mask = None
        if config['preprocessing']['yellow_bright_filter']['apply']:
            yellow_bright_mask = apply_bright_filter(frame, config['preprocessing']['yellow_bright_filter']['threshold'])
        if config['preprocessing']['white_bright_filter']['apply']:
            white_bright_mask = apply_bright_filter(frame, config['preprocessing']['white_bright_filter']['threshold'])

        # Convert to HSV
        hsv = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2HSV)

        # Create color masks
        yellow_mask = create_color_mask(hsv, config['color_ranges']['yellow'])
        white_mask = create_color_mask(hsv, config['color_ranges']['white'])

        # Apply bright masks if enabled
        if yellow_bright_mask is not None:
            yellow_mask = cv2.bitwise_and(yellow_mask, yellow_bright_mask)
        if white_bright_mask is not None:
            white_mask = cv2.bitwise_and(white_mask, white_bright_mask)

        # Combine masks
        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)
        result = cv2.bitwise_and(processed_frame, processed_frame, mask=combined_mask)

        # Publish image
        node.publish_image(result)

        # Display
        if display_mode in ['original', 'processed', 'yellow', 'white', 'combined']:
            if not display_window_exists:
                cv2.namedWindow('Display', cv2.WINDOW_NORMAL)
                display_window_exists = True
            display_img = {
                'original': frame,
                'processed': processed_frame,
                'yellow': cv2.bitwise_and(processed_frame, processed_frame, mask=yellow_mask),
                'white': cv2.bitwise_and(processed_frame, processed_frame, mask=white_mask),
                'combined': result
            }[display_mode]
            cv2.imshow('Display', display_img)
        elif display_mode == 'none' and display_window_exists:
            cv2.destroyWindow('Display')
            display_window_exists = False

        # Handle key inputs
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            save_camera_settings('src/cam_settings.yaml', config)
        elif key == ord('t'):
            trackbars_visible = not trackbars_visible
            if trackbars_visible:
                create_trackbars(config)
            else:
                cv2.destroyWindow('Trackbars')
        elif key == ord('o'):
            display_mode = 'original'
        elif key == ord('f'):
            display_mode = 'processed'
        elif key == ord('y'):
            display_mode = 'yellow'
        elif key == ord('w'):
            display_mode = 'white'
        elif key == ord('c'):
            display_mode = 'combined'
        elif key == ord('n'):
            display_mode = 'none'

        rclpy.spin_once(node, timeout_sec=0.01)

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()