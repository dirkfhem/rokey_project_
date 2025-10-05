import os
import time
import sys
from scipy.spatial.transform import Rotation
import numpy as np
import rclpy
from rclpy.node import Node
import DR_init
from copy import deepcopy

from rclpy.executors import MultiThreadedExecutor
from od_msg.srv import SrvDepthPosition
from ament_index_python.packages import get_package_share_directory
from robot_control.onrobot import RG
from std_msgs.msg import String
import tty
import termios
from robot_control.yaml_set import ConfigManager
import threading
from custom_pkg_srv_test.srv import CustomService
from robot_control.pos import POS
    
package_path = get_package_share_directory("pick_and_place_voice")


#self.pos_robot.get_pos_x("x")는 x의 posx좌표를 읽어오는데 json으로 작성된
#/home/minsuje/ros2_ws/src/DoosanBootcamp3rd/dsr_rokey/pick_and_place_voice/robot_control/data/pos/spots.json
#위주소에 저장된것을 읽어오며 해당 json을 수정되면 수정된 값을 적용하여 동작 가능.
# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0

ox, oy  = 0, 0     #객체 중앙 좌표 result 받아올 글로벌 상수 선언
quadrant = 0       # 4분면 결정 변수 0,1,2,3,4 (0:기본(분면 아님error 상황),1:1분면,2:2분면,3:3분면:4분면) 추가

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

dsr_node = rclpy.create_node("robot_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, trans, DR_MV_MOD_REL, DR_TOOL, set_singularity_handling, ikin, DR_AVOID       #이모저모 추가
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit()

########### Gripper Setup. Do not modify this area ############

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)


########### Robot Controller ############


class RobotController(Node):
    def __init__(self):
        super().__init__("pick_and_place")
        print("b")
        self.pos_robot = POS()
        print("A")
        self.get_position_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        print("c")
        while not self.get_position_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info("Waiting for get_depth_position service...")
        self.get_position_request = SrvDepthPosition.Request()
        self.status = False
        self.publishing_enabled = True
        self.orders = None
        self.num_orders = None
        self.key_sequence = []
        self.status_pub = self.create_publisher(String, '/status', 10)
        self.srv = self.create_service(CustomService, 'custom_service', self.service_callback)
        self.status_topic_timer = self.create_timer(1.0, self.publish_status)
        self.current_target = None
        self.current_count = 0
        self.target_pos = None
        self.init_robot()
        print("d")

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
        self.status = flag

    def service_callback(self, request, response):
        self.pos_robot = POS()
        self.handle_status_robot(True)
        self.orders = request.brand
        self.num_orders = request.count
        self._robot_thread = threading.Thread(target=self.robot_control, daemon=True)
        self._robot_thread.start()
        for num in range(len(request.brand)):
            self.get_logger().info(f'수신: brand={request.brand[num]}, count={request.count[num]}')
        response.success = True
        return response

    def cleanup(self):
        """터미널 설정 복원"""
        termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.old_settings)

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        """
        Converts 3D coordinates from the camera coordinate system
        to the robot's base coordinate system.
        """
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)  # Homogeneous coordinate

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)

        # 좌표 변환 (그리퍼 → 베이스)
        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3]
    


    def robot_control(self):
        for target, count in zip(self.orders, self.num_orders):
            for _ in range(count):
                target_pos = self.get_target_pos(target)
                if target_pos is None:
                    self.get_logger().warn(f"Failed to find position for {target}")
                else:
                    self.ready_for_grab(target_pos)
                    self.pick_and_place_target(target_pos)
                    self.init_robot()
        self.handle_status_robot(False)

    def get_target_pos(self, target):
        import ast
        if isinstance(target, list):        #타겟이 list타입이면 첫번째 인자 수행
            target = target[0]
        elif isinstance(target, str):       #str 타입이라면
            try:
                parsed = ast.literal_eval(target)       #리터럴하게 재평가 혹은 재 확인
                if isinstance(parsed, list):            #평가 결과 리스트라면
                    target = parsed[0]
            except (ValueError, SyntaxError):
                pass


        self.get_position_request.target = target
        self.get_logger().info(f"call depth position service with object_detection node for target: {target}")

        get_position_future = self.get_position_client.call_async(self.get_position_request)
        rclpy.spin_until_future_complete(self, get_position_future)

        if get_position_future.result():
            result = get_position_future.result().depth_position.tolist()
            self.get_logger().info(f"Received depth position: {result}")
            if sum(result) == 0:
                print("No target position")
                return None

            global ox, oy, quadrant                         #사분면 결정 저장
            ox, oy = result[0], result[1]
            if ox >= 0 and oy < 0:
                quadrant = 1
            elif ox < 0 and oy < 0:
                quadrant = 2
            elif ox < 0 and oy >= 0:
                quadrant = 3
            elif ox >= 0 and oy >= 0:
                quadrant = 4

            gripper2cam_path = os.path.join(package_path, "resource", "T_gripper2camera.npy")
            robot_posx = get_current_posx()[0]
            td_coord = self.transform_to_base(result, gripper2cam_path, robot_posx)

            if td_coord[2] and sum(td_coord) != 0:
                td_coord[2] += DEPTH_OFFSET
                td_coord[2] = max(td_coord[2], MIN_DEPTH)

            tool_coord = self.pos_robot.get_pos_x("tool_coord")
            target_pos = list(td_coord[:3]) + tool_coord        #툴 정렬 방향 결정
            return target_pos

        return None


    def init_robot(self):
        movej(self.pos_robot.get_pos_j("ready"), vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    def pick_and_place_target(self, target_pos):
        # target_pos[2] += 25
        # target_pos[0] += 30
        target_pos = [x+y for x,y in zip(target_pos, self.pos_robot.get_pos_x("grab_pos"))]
        set_singularity_handling(mode=DR_AVOID)

        movel(target_pos, vel=VELOCITY, acc=ACC)
        mwait()
        # time.sleep(5)
        gripper.close_gripper(force_val=80)         #잡기

        while gripper.get_status()[0]:
            time.sleep(0.1)
        while not gripper.get_status()[1]:
            self.get_logger().info("not grabbed")

        movel(
            self.pos_robot.get_pos_x("go_up"), 
              vel=VELOCITY, 
              acc=ACC, 
              mod=DR_MV_MOD_REL
              )
        mwait()


        movel(self.pos_robot.get_pos_x("go_up2"),                 # 툴좌표 z 이동
            vel=VELOCITY,
            acc=ACC,
            ref=DR_TOOL,
            mod=DR_MV_MOD_REL,
        )
        mwait()

    def ready_for_grab(self, target_pos):
        sol = 0
        above_target = deepcopy(target_pos)
        above_target = [x+y for x, y in zip(above_target,self.pos_robot.get_pos_x("ready_for_coor"))]
        # above_target[0]-=20
        # above_target[2]+=20
        
        if quadrant == 1:
            sol = 2 
            self.get_logger().info("sol == 2")
        else:
            sol = 3
            self.get_logger().info("sol == 3")
        if sol == 0:
            self.get_logger().info("죽어")
            return
        above_target_j = ikin(above_target, sol).tolist()       #좌측 3 우측 2
        #bove_target_j = ikin(above_target, sol_space=3).tolist()       #좌측 3 우측 2
        # self.mj.move_j(above_target_j, vel=30, acc=30)
        cp1 = get_current_posx()[0]
        movej(above_target_j, vel=VELOCITY, acc=ACC)
        cp2 = get_current_posx()[0]
    
        if cp1 == cp2:
            self.get_logger().info("j이동 실패")
        else:
            self.get_logger().info("j이동 완료")

        # time.sleep(5)


def main(args=None):
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == "__main__":
    main()
    