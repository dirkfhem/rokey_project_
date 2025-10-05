import rclpy
import DR_init
import numpy as np
import json

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init(args=None)
node = rclpy.create_node("pos", namespace=ROBOT_ID)

DR_init.__dsr__node = node

from DSR_ROBOT2 import (
    ikin,
    DR_BASE,
    fkin,
)
POS_X = "0"
POS_J = "1"

''' 
file_path는 각자 맞는거로 수정만 하면되고 
처음 POS 쓸대만 딕셔너리 구조 {"name": {POS_X: [x,y,z,a,b,c], POS_J: [j1,j2,j3,j4,j5,j6]}} 이렇게 보내면 되는데 
posx든 posj든 이름이랑 같이 해서 내면됨 {"name": {POS_X: [x,y,z,a,b,c]}} 이렇게나 {"name": {POS_J: [j1,j2,j3,j4,j5,j6]}}
참고로 POS_X와 POS_J 정의는 위에 있으니까 그대로 들고가거나 그냥 저렇게 넣어도됨.
이름만 넣으면 에러로 뜰거임. X,J 둘다 넣으면 그냥 그대로 알아서 저장해줌
이후에 쓸때는 그냥 POS(file_path)만하면 알아서 그파일 페턴에서 읽어와서 쓸수 있는데 예를 들자면
ps = POS(file_path=path)

ps.get_pos_x("name")
ps.get_pos_j("name")
이런식으로 쓰면 직관적으로 j좌표이구나 x좌표이구나 구분되고 이름을 자세히 쓸수록 음 뭘 하는 좌표구나 하면서 해석 하기 편하겠죠? 굿 그렇게 쓰면 됩니다.

참고로 자동계산 x는 아주 잘 나오는데 j는 solspace에 따라 자세가 안나오거나 아예 없거나 있어도 동작은 안될 수 있음을 참조 해주시구요 직접 실행해보고 
 def set_j(self, pos_x, solspace=4): 에서 solspace=4에 숫자를 0~7로 바꿔서 직접 이동 시켜보고 판단도 해야 할 수 있어요.
뭐 딱히 어느 방향으로 j가 움직이든 별 상관 없으면 막 써도되는데 아니면 생각과 다르게 움직일 수 있어요 그래서 j는 직접 찾아오는게 더 좋을 거임.
'''

class POS:
    def __init__(self, pos_dict=None, file_path="src/DoosanBootcamInt1/dsr_rokey/rokey/rokey/setpy/data/pos/spots.json"):
        self.spots = {}  # {"name": {POS_X: [x,y,z,a,b,c], POS_J: [j1,j2,j3,j4,j5,j6]}}
        self.file_path = file_path

        # pos_dict가 제공되면 유효성 검사 후 처리 및 저장
        if pos_dict is not None:
            if not isinstance(pos_dict, dict):
                print(f"Error: pos_dict must be a dictionary, got {type(pos_dict)}")
            elif not pos_dict:
                print("Warning: pos_dict is empty, initializing with empty spots")
            else:
                self.add_spots_from_dict(pos_dict)
                self.save_spots()
        # pos_dict가 없으면 파일에서 로드 시도
        else:
            print("pos를 읽어오는 중입니다.")
            self.load_spots()
    
    def set_x(self, pos_j, ref=DR_BASE):
        """pos_j를 입력받아 pos_x(카르테시안 좌표)를 계산"""
        try:
            x = fkin(pos_j, ref)
            return x.tolist() if isinstance(x, np.ndarray) else x
        except Exception as e:
            print(f"fkin error: {e}")
            return None

    def set_j(self, pos_x, solspace=4):
        """pos_x를 입력받아 pos_j(관절 좌표)를 계산"""
        try:
            j = ikin(pos_x, solspace)
            return j.tolist() if isinstance(j, np.ndarray) else j
        except Exception as e:
            print(f"ikin error: {e}")
            return None

    def add_spot(self, name, pos_x=None, pos_j=None, solspace=4):
        """이름과 pos_x 또는 pos_j (또는 둘 다)를 받아 스팟 추가"""
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
        elif pos_x is not None:
            # pos_x만 제공된 경우: pos_j 계산
            pos_j = self.set_j(pos_x, solspace)
            if pos_j is None:
                print(f"Failed to calculate pos_j for {name}")
                return False
            self.spots[name] = {POS_X: pos_x, POS_J: pos_j}
            print(f"Added spot {name}: pos_x={pos_x}, pos_j={pos_j} (calculated pos_j)")
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
            if not isinstance(data, dict):
                print(f"Error: Data for {name} must be a dictionary, got {type(data)}")
                continue
            pos_x = data.get(POS_X)
            pos_j = data.get(POS_J)
            if not self.add_spot(name, pos_x=pos_x, pos_j=pos_j):
                print(f"Failed to add spot {name}")

    def add_spots_from_list(self, pos_list, name_prefix="pos"):
        """리스트 형태 [[x,y,z,a,b,c], ...]로 여러 스팟 추가"""
        for i, pos_x in enumerate(pos_list):
            name = f"{name_prefix}_{i+1}_spot"
            self.add_spot(name, pos_x=pos_x)

    def get_pos_x(self, name):
        """이름으로 pos_x 조회"""
        spot = self.spots.get(name)
        if spot is None:
            print(f"Spot {name} not found")
            return None
        return spot[POS_X]

    def get_pos_j(self, name):
        """이름으로 pos_j 조회"""
        spot = self.spots.get(name)
        if spot is None:
            print(f"Spot {name} not found")
            return None
        return spot[POS_J]

    def get_spot(self, name):
        """이름으로 pos_x와 pos_j 조회"""
        spot = self.spots.get(name)
        if spot is None:
            print(f"Spot {name} not found")
            return None
        return spot[POS_X], spot[POS_J]

    def get_all_spots(self):
        """모든 스팟 정보 리턴"""
        return self.spots

    def save_spots(self):
        """spots 데이터를 JSON 파일로 저장"""
        try:
            with open(self.file_path, "w") as f:
                json.dump(self.spots, f, indent=4)
            print(f"Spots saved to {self.file_path}")
        except Exception as e:
            print(f"Error saving spots to {self.file_path}: {e}")

    def load_spots(self):
        """JSON 파일에서 spots 데이터 로드"""
        try:
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