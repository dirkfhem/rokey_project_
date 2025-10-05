import rclpy
import DR_init
import math
import time

# Existing imports and constants remain unchanged
# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 상수 및 초기 데이터
NUM_PLATES = 2  # 플레이트 수, 변경 가능
PLATE_POSITIONS = [
    ([497.04, 152.34, 130.24, 1.94, -179.83, 2.23], [599.04, 48.95, 130.24, 1.94, -179.83, 2.23]),  # 플레이트 0: 위치 0, 위치 8
    ([495.83, -49.11, 130.24, 1.94, -179.83, 2.23], [598.1, -151.73, 130.24, 1.94, -179.83, 2.23])   # 플레이트 1: 위치 0, 위치 8
]
home = [366.42, 3.72, 194.39, 179.82, 179.82, -179.79]

GRIP = 1
RELEASE = 2

ON_ = 1
OFF_ = 0

POS_J = 1
POS_X = 0

DESC = 0  # 내림차순: 0~8, 높은 순부터 낮은 순
ASCE = 1  # 오름차순: 0~8, 낮은 순부터 높은 순

DR_ACC_L = 70
DR_VEL_L = 70
DR_ACC_J = 70
DR_VEL_J = 70

EXIST = 1
NOTHING = 0

PICK_UP = False
PICK_DOWN = True

rclpy.init(args=None)
node = rclpy.create_node("rokey_simple_move", namespace=ROBOT_ID)
DR_init.__dsr__node = node

from DSR_ROBOT2 import (
    set_tool,
    set_tcp,
    movej,
    movel,
    ikin,
    set_digital_output,
    get_current_posx,
    set_desired_force,
    task_compliance_ctrl,
    check_force_condition,
    release_force,
    release_compliance_ctrl,
    get_tool_force,
    wait,
    DR_BASE,
    DR_TOOL,
    DR_AXIS_Z,
)

from DR_common2 import posx, posj

set_tool("TCP208mm")
set_tcp("GripperSA_rg2_250509")

class plate_control:
    def __init__(self, plate_positions, args=None):
        if len(plate_positions) < 1:
            raise ValueError("최소한 하나의 플레이트가 지정되어야 합니다")
        self.num_plates = len(plate_positions)
        self.plate = [[None for _ in range(9)] for _ in range(self.num_plates)]  # [플레이트][위치] = [posx, posj]
        self.exist_is = [[NOTHING] * 9 for _ in range(self.num_plates)]  # [플레이트][위치] = EXIST 또는 NOTHING
        self.home_m_height = [0.0] * 9  # 플레이트 0 (홈)의 높이
        self.home_m_weight = [0.0] * 9  # 플레이트 0 (홈)의 무게
        # 플레이트 0을 아이템으로 초기화
        self.exist_is[0] = [EXIST] * 9
        self.down_h = 70
        # 각 플레이트의 좌표 계산
        for plate_idx, (pos0, pos8) in enumerate(plate_positions):
            self.calculate_plate_coor(plate_idx, pos0, pos8)

    def griper(self):
        task_compliance_ctrl(stx=[2000, 2000, 100, 20, 20, 20])
        time.sleep(0.1)
        set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
        
        while True:
            if check_force_condition(axis=DR_AXIS_Z, max=12, ref=DR_TOOL):
                task_compliance_ctrl(stx=[2000, 2000, 100, 20, 20, 20])
                set_desired_force(fd=[0, 0, 10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
                time.sleep(0.3)
                release_force()
                release_compliance_ctrl()
                set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
                wait(0.3)
                self.release()
                release_force()
                release_compliance_ctrl()
                self.grip()
                self.down_h = get_current_posx()[0][2]
                break

    def grip(self):
        set_digital_output(GRIP, ON_)
        wait(1.0)
        set_digital_output(GRIP, OFF_)

    def release(self):
        set_digital_output(RELEASE, ON_)
        wait(1.0)
        set_digital_output(RELEASE, OFF_)

    def movel_down(self, pos_x, flag):
        new_pos_x = posx(pos_x)
        if flag == True:
            new_pos_x[2] = self.down_h
        else:
            new_pos_x[2] = 70.0
        movel(new_pos_x, vel=DR_VEL_L, acc=DR_ACC_L)

    def set_j(self, l):
        """
        posx를 posj로 변환하는 함수 (ikin 사용).
        posx: [x, y, z, rx, ry, rz]
        반환: [j1, j2, j3, j4, j5, j6] 또는 해결책이 없으면 None
        """
        try:
            sol_spaces = [2, 0, 4]
            ref = DR_BASE
            ref_pos_opt = 0
            iter_threshold = [0.005, 0.01]
            for sol_space in sol_spaces:
                j, status = ikin(l, sol_space, ref, ref_pos_opt, iter_threshold)
                if status == 0 and j is not None and len(j) == 6:
                    return j.tolist()
            return None
        except Exception:
            return None

    def calculate_plate_coor(self, plate_idx, poly0):
        x0, y0, z0, rx0, ry0, rz0 = poly0
        dl = 51
        coordinates = []
        for i in range(3):
            for j in range(3):
                x = x0 + j * dl
                y = y0 - i * dl
                pos_x = [x, y, z0, rx0, ry0, rz0]
                pos_j = self.set_j(pos_x)
                if pos_j is None:
                    pos_j = [0.0] * 6
                coordinates.append([pos_x, pos_j])
        for idx, coord in enumerate(coordinates):
            self.plate[plate_idx][idx] = coord

    def get_coordinate(self, plate_idx, pos_idx, coord_type=0):
        if 0 <= plate_idx < self.num_plates and 0 <= pos_idx < 9:
            return self.plate[plate_idx][pos_idx][coord_type]
        return None

    def measure_height(self, plate_idx, pos_idx):
        """
        Measure the height of an item at the specified plate and position.
        Returns the height (z-coordinate) from griper's down_h.
        """
        if self.exist_is[plate_idx][pos_idx] == NOTHING:
            return 0.0
        poly = self.get_coordinate(plate_idx, pos_idx, POS_X)
        if poly is None:
            return 0.0
        movel(poly, vel=DR_VEL_L, acc=DR_ACC_L)
        self.movel_down(poly, PICK_UP)
        self.griper()
        height = self.down_h
        movel(poly, vel=DR_VEL_L, acc=DR_ACC_L)
        self.movel_down(poly, PICK_DOWN)
        self.release()
        movel(poly, vel=DR_VEL_L, acc=DR_ACC_L)
        self.grip()
        return height

    def measure_all_heights(self):
        """
        Measure heights of all items on the home plate (plate 0) and store in home_m_height.
        """
        for pos_idx in range(9):
            if self.exist_is[0][pos_idx] == EXIST:
                self.home_m_height[pos_idx] = self.measure_height(0, pos_idx)
            else:
                self.home_m_height[pos_idx] = 0.0

    def move_item(self, from_plate, from_idx, to_plate, to_idx):
        # from_plate[from_idx]에서 to_plate[to_idx]로 아이템 이동
        if (0 <= from_plate < self.num_plates and 0 <= to_plate < self.num_plates and
            0 <= from_idx < 9 and 0 <= to_idx < 9 and
            self.exist_is[from_plate][from_idx] == EXIST and
            self.exist_is[to_plate][to_idx] == NOTHING):
            poly = self.plate[from_plate][from_idx]
            target_poly = self.plate[to_plate][to_idx]
            
            # 원래 위치에서 아이템 집기
            movel(poly[POS_X], vel=DR_VEL_J, acc=DR_ACC_J)
            self.movel_down(poly[POS_X],PICK_UP)
            self.griper()
            movel(poly[POS_X], ref=DR_BASE, vel=DR_VEL_L, acc=DR_ACC_L)
            
            # 목표 위치에 아이템 놓기
            movej(target_poly[POS_J], vel=DR_VEL_J, acc=DR_ACC_J)
            self.movel_down(target_poly[POS_X],PICK_DOWN)
            self.release()
            movel(target_poly[POS_X], ref=DR_BASE, vel=DR_VEL_L, acc=DR_ACC_L)
            self.grip()
            
            # 플레이트와 존재 여부 업데이트
            self.plate[to_plate][to_idx] = poly
            self.plate[from_plate][from_idx] = None
            self.exist_is[from_plate][from_idx] = NOTHING
            self.exist_is[to_plate][to_idx] = EXIST
            if from_plate == 0:
                self.home_m_weight[from_idx] = 0.0
            return True
        return False

    def sort_by_height(self, sort=DESC):
        """
        Sort items on the home plate by height.
        sort: DESC (0) for high to low, ASCE (1) for low to high.
        """
        # Measure heights for all positions on home plate
        self.measure_all_heights()

        # Create list of (height, index) for items that exist
        height_indices = [(height, idx) for idx, height in enumerate(self.home_m_height)
                         if self.exist_is[0][idx] == EXIST]
    
        # Categorize heights into three ranges
        heights = [h for h, _ in height_indices]
        if not heights:
            return  # No items to sort
        min_height, max_height = min(heights), max(heights)
        if max_height == min_height:
            range_size = 1.0
        else:
            range_size = (max_height - min_height) / 3.0
        low_threshold = min_height + range_size
        high_threshold = min_height + 2 * range_size

        def categorize_height(height):
            if height < low_threshold:
                return 0  # Low
            elif height < high_threshold:
                return 1  # Medium
            else:
                return 2  # High
        
        
        # Sort by height and category
        height_indices.sort(key=lambda x: (categorize_height(x[0]), x[0]), reverse=(sort == DESC))

        # Find empty storage positions
        empty_storage = []
        for plate_idx in range(1, self.num_plates):
            for pos_idx in range(9):
                if self.exist_is[plate_idx][pos_idx] == NOTHING:
                    empty_storage.append((plate_idx, pos_idx))

        # Temporary storage for home plate
        temp_plate = [None] * 9
        temp_heights = [0.0] * 9
        temp_exist = [NOTHING] * 9

        # Move items to storage temporarily
        storage_pos = 0
        for _, home_idx in height_indices:
            if storage_pos < len(empty_storage):
                plate_idx, pos_idx = empty_storage[storage_pos]
                self.move_item(0, home_idx, plate_idx, pos_idx)
                storage_pos += 1

        # Move items back to home plate in sorted order
        for new_idx, (height, _) in enumerate(height_indices):
            if storage_pos > 0:
                storage_pos -= 1
                plate_idx, pos_idx = empty_storage[storage_pos]
                self.move_item(plate_idx, pos_idx, 0, new_idx)
                temp_plate[new_idx] = self.plate[0][new_idx]
                temp_heights[new_idx] = height
                temp_exist[new_idx] = EXIST

        # Update home plate
        self.plate[0] = temp_plate
        self.home_m_height = temp_heights
        self.exist_is[0] = temp_exist

    def weight_measurment(self, poly):
        movel(poly[POS_X], vel=DR_VEL_L, acc=DR_ACC_L)
        self.movel_down(poly[POS_X], PICK_UP)
        self.griper()
        movel(poly[POS_X], vel=DR_VEL_L, acc=DR_ACC_L)
        wait(0.5)
        force = get_tool_force(DR_BASE)
        force_magnitude = math.sqrt(force[0]**2 + force[1]**2 + force[2]**2)
        result = force_magnitude / 9.81
        wait(0.5)
        self.movel_down(poly[POS_X], PICK_DOWN)
        self.release()
        wait(0.5)
        movel(poly[POS_X], vel=DR_VEL_L, acc=DR_ACC_L)
        self.grip()
        return result

    def sort_by_weight(self, sort=DESC):
        weight_indices = [(weight, idx) for idx, weight in enumerate(self.home_m_weight)
                         if self.exist_is[0][idx] == EXIST]
        if sort == DESC:
            weight_indices.sort(reverse=True)
        else:
            weight_indices.sort()
        empty_storage = []
        for plate_idx in range(1, self.num_plates):
            for pos_idx in range(9):
                if self.exist_is[plate_idx][pos_idx] == NOTHING:
                    empty_storage.append((plate_idx, pos_idx))
        temp_plate = [None] * 9
        temp_weights = [0.0] * 9
        temp_exist = [NOTHING] * 9
        storage_pos = 0
        for _, home_idx in weight_indices:
            if storage_pos < len(empty_storage):
                plate_idx, pos_idx = empty_storage[storage_pos]
                self.move_item(0, home_idx, plate_idx, pos_idx)
                storage_pos += 1
        for new_idx, (weight, _) in enumerate(weight_indices):
            if storage_pos > 0:
                storage_pos -= 1
                plate_idx, pos_idx = empty_storage[storage_pos]
                self.move_item(plate_idx, pos_idx, 0, new_idx)
                temp_plate[new_idx] = self.plate[0][new_idx]
                temp_weights[new_idx] = weight
                temp_exist[new_idx] = EXIST
        self.plate[0] = temp_plate
        self.home_m_weight = temp_weights
        self.exist_is[0] = temp_exist

def main(sort_order=DESC, use_sequence=None):
    """
    Main function to initialize and sort items by height.
    sort_order: DESC (0) for high to low, ASCE (1) for low to high.
    use_sequence: Optional string of position indices (e.g., "048237156") to move items before sorting.
    """
    try:
        # Initialize plate control
        pc = plate_control(PLATE_POSITIONS)
        
        # Move to home position
        movel(home, vel=DR_VEL_J, acc=DR_ACC_J)

        print("do self sort")
        # Sort by height

        pc.sort_by_height(sort_order)
        print(f"Items sorted by height in {'descending' if sort_order == DESC else 'ascending'} order")
        
        # Move back to home position
        movel(home, vel=DR_VEL_J, acc=DR_ACC_J)
        
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()  

    35, 55, 45