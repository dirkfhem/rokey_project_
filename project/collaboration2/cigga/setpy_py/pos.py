import rclpy
import DR_init
import numpy as np
import json
import os
import sys
sys.path.append(os.path.expanduser('~/ros2_ws/install/common2/lib/common2/imp'))

# Robot configuration
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

try:
    rclpy.init()
    node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
except Exception as e:
    print(f"Error initializing ROS2: {e}")
    exit(1)

from DSR_ROBOT2 import (
    ikin,
    DR_BASE,
    fkin,
)

# Constants
POS_X = "0"
POS_J = "1"
SECTION_1 = "0"
SECTION_2 = "1"

class POS:
    def __init__(self, pos_dict=None, file_path="/home/minsuje/ros2_ws/setpy_py/data/pos/spots.json"):
        self.spots = {}  # {"name": {POS_X: [x,y,z,a,b,c], POS_J: [j1,j2,j3,j4,j5,j6]}}
        self.file_path = file_path

        # Ensure directory exists
        os.makedirs(os.path.dirname(file_path), exist_ok=True)
        print(f"Ensured directory exists for {file_path}")

        # pos_dict가 제공되면 유효성 검사 후 처리 및 저장
        if pos_dict is not None:
            if not isinstance(pos_dict, dict):
                print(f"Error: pos_dict must be a dictionary, got {type(pos_dict)}")
            elif not pos_dict:
                print("Warning: pos_dict is empty, initializing with empty spots")
            else:
                print("Adding spots from dictionary...")
                self.add_spots_from_dict(pos_dict)
                self.save_spots()
        else:
            print("pos를 읽어오는 중입니다.")
            self.load_spots()
    
    def set_x(self, pos_j, ref=DR_BASE):
        """pos_j를 입력받아 pos_x(카르테시안 좌표)를 계산"""
        try:
            print(f"Calculating pos_x for pos_j={pos_j}")
            x = fkin(pos_j, ref)
            result = x.tolist() if isinstance(x, np.ndarray) else x
            print(f"Calculated pos_x={result}")
            return result
        except Exception as e:
            print(f"fkin error: {e}")
            return None

    def set_j(self, pos_x, solspace=4):
        """pos_x를 입력받아 pos_j(관절 좌표)를 계산"""
        try:
            print(f"Calculating pos_j for pos_x={pos_x}, solspace={solspace}")
            j = ikin(pos_x, solspace)
            result = j.tolist() if isinstance(j, np.ndarray) else j
            print(f"Calculated pos_j={result}")
            return result
        except Exception as e:
            print(f"ikin error for pos_x={pos_x}, solspace={solspace}: {e}")
            return None

    def add_spot(self, name, pos_x=None, pos_j=None, solspace=4, require_pos_j=True):
        """이름과 pos_x 또는 pos_j (또는 둘 다)를 받아 스팟 추가"""
        print(f"Attempting to add spot {name}")
        # 입력 검증
        if pos_x is not None and (not isinstance(pos_x, list) or len(pos_x) != 6):
            print(f"Error: pos_x {pos_x} must be a list of 6 elements [x,y,z,a,b,c]")
            return False
        if pos_j is not None and (not isinstance(pos_j, list) or len(pos_j) != 6):
            print(f"Error: pos_j {pos_j} must be a list of 6 elements [j1,j2,j3,j4,j5,j6]")
            return False
        if pos_x is None and pos_j is None:
            print(f"Error: At least one of pos_x or pos_j must be provided for {name}")
            return False

        # 경우에 따라 처리
        if pos_x is not None and pos_j is not None:
            # 둘 다 제공된 경우: 변환 없이 저장
            self.spots[name] = {POS_X: pos_x, POS_J: pos_j}
            print(f"Added spot {name}: pos_x={pos_x}, pos_j={pos_j} (no conversion)")
        elif pos_x is not None and require_pos_j:
            # pos_x만 제공되고 POS_J가 필요한 경우: pos_j 계산
            for sol in range(8):
                pos_j = self.set_j(pos_x, solspace=sol)
                if pos_j is not None:
                    self.spots[name] = {POS_X: pos_x, POS_J: pos_j}
                    print(f"Added spot {name}: pos_x={pos_x}, pos_j={pos_j} (calculated pos_j with solspace={sol})")
                    return True
            print(f"Failed to calculate pos_j for {name} with any solspace")
            return False
        elif pos_x is not None:
            # pos_x만 제공되고 POS_J가 필요 없는 경우: pos_x만 저장
            self.spots[name] = {POS_X: pos_x}
            print(f"Added spot {name}: pos_x={pos_x} (no pos_j required)")
        else:
            # pos_j만 제공된 경우: pos_x 계산
            pos_x = self.set_x(pos_j)
            if pos_x is None:
                print(f"Failed to calculate pos_x for {name}")
                return False
            self.spots[name] = {POS_X: pos_x, POS_J: pos_j}
            print(f"Added spot {name}: pos_x={pos_x}, pos_j={pos_j} (calculated pos_x)")

        return True

    def add_spots_from_dict(self, pos_dict):
        """딕셔너리 형태로 여러 스팟 추가"""
        for name, data in pos_dict.items():
            print(f"Processing spot {name}")
            if not isinstance(data, dict):
                print(f"Error: Data for {name} must be a dictionary, got {type(data)}")
                continue
            pos_x = data.get(POS_X)
            pos_j = data.get(POS_J)
            # Only require POS_J for 'ready' spot
            require_pos_j = (name == "ready")
            if not self.add_spot(name, pos_x=pos_x, pos_j=pos_j, require_pos_j=require_pos_j):
                print(f"Failed to add spot {name}")

    def add_spots_from_list(self, pos_list, name_prefix="pos"):
        """리스트 형태 [[x,y,z,a,b,c], ...]로 여러 스팟 추가"""
        for i, pos_x in enumerate(pos_list):
            name = f"{name_prefix}_{i+1}_spot"
            self.add_spot(name, pos_x=pos_x, require_pos_j=False)

    def get_pos_x(self, name):
        """이름으로 pos_x 조회"""
        spot = self.spots.get(name)
        if spot is None:
            print(f"Spot {name} not found")
            return None
        return spot.get(POS_X)

    def get_pos_j(self, name):
        """이름으로 pos_j 조회"""
        spot = self.spots.get(name)
        if spot is None:
            print(f"Spot {name} not found")
            return None
        return spot.get(POS_J)

    def get_spot(self, name):
        """이름으로 pos_x와 pos_j 조회"""
        spot = self.spots.get(name)
        if spot is None:
            print(f"Spot {name} not found")
            return None
        return spot.get(POS_X), spot.get(POS_J)

    def get_all_spots(self):
        """모든 스팟 정보 리턴"""
        return self.spots

    def save_spots(self):
        """spots 데이터를 JSON 파일로 저장"""
        try:
            print(f"Saving spots to {self.file_path}")
            with open(self.file_path, "w") as f:
                json.dump(self.spots, f, indent=4)
            print(f"Spots saved to {self.file_path}")
        except Exception as e:
            print(f"Error saving spots to {self.file_path}: {e}")

    def load_spots(self):
        """JSON 파일에서 spots 데이터 로드"""
        try:
            print(f"Loading spots from {self.file_path}")
            with open(self.file_path, "r") as f:
                self.spots = json.load(f)
            print(f"Spots loaded from {self.file_path}")
            print(f"spot={self.spots}")
        except FileNotFoundError:
            print(f"Error: File {self.file_path} not found")
            self.spots = {}
        except Exception as e:
            print(f"Error loading spots from {self.file_path}: {e}")
            self.spots = {}

coordinates = {
    "ready": {
        POS_J: [-3.57, -50.24, 107.06, -4.6, 52.94, 92.02],
        POS_X: [328.15, -41.05, 476.22, 172.35, -109.66, -91.64]
    },
    "ready_for_coor": {
        POS_X: [-20, 0, 20, 0, 0, 0]
    },
    "grab_pos": {
        POS_X: [30, 0, 25, 0, 0, 0]
    },
    "go_up": {
        POS_X: [0, 0, 60, 0, 0, 0]
    },
    "go_up2": {
        POS_X: [0, 0, -100, 0, 0, 0]
    },
    "tool_coord": {
        POS_X: [0, 0, 0, 178.38, -90.11, -89.87]
    }
}

# Initialize POS class and add coordinates
def main():
    file_path = "/home/minsuje/ros2_ws/setpy_py/data/pos/spots.json"
    print("Starting POS initialization...")
    pos_manager = POS(pos_dict=coordinates, file_path=file_path)
    print("\nStored spots:")
    print(pos_manager.get_all_spots())
    
    # Example of calculating target_pos
    td_coord = [100, 200, 300, 0, 0, 0]  # Example td_coord
    tool_coord = pos_manager.get_pos_x("tool_coord")
    if tool_coord:
        target_pos = list(td_coord[:3]) + tool_coord[3:]
        print("\nCalculated target_pos:")
        print(target_pos)

if __name__ == "__main__":
    main()