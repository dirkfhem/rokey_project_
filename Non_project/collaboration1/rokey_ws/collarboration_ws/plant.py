# 상수 및 초기 데이터
NUM_PLATES = 2  # 플레이트 수, 변경 가능
# 9개의 위치가 아닌 시작과 끝점을 통해 나머지 7개 위치 자동 계산 하기 위해 두개씩만 함.
PLATE_POSITIONS = [
    ([497.04, 152.34, 120.24, 1.94, -179.83, 2.23], [599.04, 48.95, 120.24, 1.94, -179.83, 2.23]),  # 플레이트 0: 위치 0, 위치 8
    ([495.83, -49.11, 120.24, 1.94, -179.83, 2.23], [598.1, -151.73, 120.24, 1.94, -179.83, 2.23])   # 플레이트 1: 위치 0, 위치 8
]


home = [366.42, 3.72, 194.39, 179.82, 179.82, -179.79]

GRIP = 1
RELEASE = 2

ON_ = 1
OFF_ = 0

POS_J = 1
POS_X = 0

DESC = 0  # 내림차순: 0~8, 무거운 순부터 가벼운 순
ASCE = 1  # 오름차순: 0~8, 가벼운 순부터 무거운 순

DR_ACC_L = 40
DR_VEL_L = 40
DR_ACC_J = 20 
DR_VEL_J = 20

EXIST = 1
NOTHING = 0

PICK_UP=False
PICK_DOWN=True

class plate_control:
    def __init__(self, plate_positions):
        if len(plate_positions) < 1:
            raise ValueError("최소한 하나의 플레이트가 지정되어야 합니다")
        self.num_plates = len(plate_positions)
        self.plate = [[None for _ in range(9)] for _ in range(self.num_plates)]  # [플레이트][위치] = [posx, posj]
        self.exist_is = [[NOTHING] * 9 for _ in range(self.num_plates)]  # [플레이트][위치] = EXIST 또는 NOTHING
        self.home_m_weight = [0.0] * 9  # 플레이트 0 (홈)의 무게
        # 플레이트 0을 아이템으로 초기화
        self.exist_is[0] = [EXIST] * 9
        self.down_h=70
        # 각 플레이트의 좌표 계산
        for plate_idx, (pos0, pos8) in enumerate(plate_positions):
            self.calculate_plate_coor(plate_idx, pos0, pos8)
    def griper(self):
        task_compliance_ctrl([2000, 2000, 100, 20, 20, 20])
        set_desired_force([0, 0, -15, 0, 0, 0], [0, 0, 1, 0, 0, 0])
        while True:
            if check_force_condition(axis=DR_AXIS_Z, min=10, ref=DR_TOOL):
                self.release()
                release_force()
                release_compliance_ctrl()
                self.grip()
                self.down_h=get_current_posx()[0][2]
                break
    def grip(self):
        set_digital_output(GRIP, ON_)
        wait(1.0)
        set_digital_output(GRIP, OFF_)

    def release(self):
        set_digital_output(RELEASE, ON_)
        wait(1.0)
        set_digital_output(RELEASE, OFF_)

    def movel_down(self, pos_x,flag):
        new_pos_x= posx(pos_x)
        if(flag==True):
            new_pos_x[2] = self.down_h
        else:
            new_pos_x[2] = 70.0
        movel(new_pos_x, ref=DR_BASE, vel=DR_VEL_L, acc=DR_ACC_L)

    # l(X-Y-Z좌표계)를 입력시 joint 좌표계로 변경하여 리턴.
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
                    return j
            return None
        except Exception:
            return None

    def calculate_plate_coor(self, plate_idx, poly0, poly8):
        # poly0: 위치 0 좌표, poly8: 위치 8 좌표
        # x, y는 보간에 사용, z, rx, ry, rz는 고정 유지
        x0, y0, z0, rx0, ry0, rz0 = poly0
        x8, y8, z8, rx8, ry8, rz8 = poly8

        # 3x3 격자의 51mm 간격
        dl = 51

        # 3x3 격자 좌표 생성
        coordinates = []
        for i in range(3):
            for j in range(3):
                x = x0 + j * dl
                y = y0 - i * dl  # y는 음의 방향으로 이동
                pos_x = [x, y, z0, rx0, ry0, rz0]
                pos_j = self.set_j(pos_x)  # 관절 좌표로 변환
                if pos_j is None:
                    pos_j = [0.0] * 6  # 해결책이 없으면 기본값
                coordinates.append([pos_x, pos_j])

        # plate[plate_idx]에 저장
        for idx, coord in enumerate(coordinates):
            self.plate[plate_idx][idx] = coord

    def get_coordinate(self, plate_idx, pos_idx, coord_type=0):
        # coord_type: 0은 posx, 1은 posj
        if 0 <= plate_idx < self.num_plates and 0 <= pos_idx < 9:
            return self.plate[plate_idx][pos_idx][coord_type]
        return None

    def weight_measurement_manger(self, sort=DESC):

        for index in range(9):
            if self.exist_is[0][index] == EXIST:
                poly = self.plate[0][index]
                self.home_m_weight[index] = self.weight_measurment(poly)
            else:
                self.home_m_weight[index] = 0.0
        self.sort_by_weight(sort)

    def move_item(self, from_plate, from_idx, to_plate, to_idx):
        # from_plate[from_idx]에서 to_plate[to_idx]로 아이템 이동
        if (0 <= from_plate < self.num_plates and 0 <= to_plate < self.num_plates and
            0 <= from_idx < 9 and 0 <= to_idx < 9 and
            self.exist_is[from_plate][from_idx] == EXIST and
            self.exist_is[to_plate][to_idx] == NOTHING):
            poly = self.plate[from_plate][from_idx]
            target_poly = self.plate[to_plate][to_idx]
            
            # 원래 위치에서 아이템 집기
            movej(poly[POS_J], vel=DR_VEL_J, acc=DR_ACC_J)
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

    def sort_by_weight(self, sort=DESC):
        # 아이템이 있는 위치에 대해 (무게, 인덱스) 튜플 리스트 생성
        weight_indices = [(weight, idx) for idx, weight in enumerate(self.home_m_weight)
                         if self.exist_is[0][idx] == EXIST]
        
        # 무게 기준으로 인덱스 정렬
        if sort == DESC:
            weight_indices.sort(reverse=True)  # 무거운 순부터 가벼운 순
        else:
            weight_indices.sort()  # 가벼운 순부터 무거운 순

        # 홈 플레이트를 제외한 모든 스토리지 플레이트에서 빈 위치 찾기
        empty_storage = []
        for plate_idx in range(1, self.num_plates):
            for pos_idx in range(9):
                if self.exist_is[plate_idx][pos_idx] == NOTHING:
                    empty_storage.append((plate_idx, pos_idx))
        
        # 정렬 중 아이템을 임시로 저장할 공간
        temp_plate = [None] * 9
        temp_weights = [0.0] * 9
        temp_exist = [NOTHING] * 9

        # 아이템을 임시로 스토리지로 이동
        storage_pos = 0
        for _, home_idx in weight_indices:
            if storage_pos < len(empty_storage):
                plate_idx, pos_idx = empty_storage[storage_pos]
                self.move_item(0, home_idx, plate_idx, pos_idx)
                storage_pos += 1

        # 아이템을 정렬된 순서로 홈 플레이트로 이동
        for new_idx, (weight, _) in enumerate(weight_indices):
            if storage_pos > 0:
                storage_pos -= 1
                plate_idx, pos_idx = empty_storage[storage_pos]
                self.move_item(plate_idx, pos_idx, 0, new_idx)
                temp_plate[new_idx] = self.plate[0][new_idx]
                temp_weights[new_idx] = weight
                temp_exist[new_idx] = EXIST

        # 홈 플레이트와 무게 업데이트
        self.plate[0] = temp_plate
        self.home_m_weight = temp_weights
        self.exist_is[0] = temp_exist

    def weight_measurment(self, poly):
        movel(poly[POS_X], ref=DR_BASE, vel=DR_VEL_L, acc=DR_ACC_L)
        self.movel_down(poly[POS_X],PICK_UP)
        self.griper()
        movel(poly[POS_X], ref=DR_BASE, vel=DR_VEL_L, acc=DR_ACC_L)
        wait(0.5)
        force = get_tool_force(DR_BASE)
        # 힘 벡터의 노름 계산: sqrt(fx^2 + fy^2 + fz^2)
        force_magnitude = math.sqrt(force[0]**2 + force[1]**2 + force[2]**2)
        result = force_magnitude / 9.81
        wait(0.5)
        self.movel_down(poly[POS_X],PICK_DOWN)
        self.release()
        wait(0.5)
        movel(poly[POS_X], ref=DR_BASE, vel=DR_VEL_L, acc=DR_ACC_L)
        server_socket_write(sock, b"here.")
        self.grip()
        return result
    

def start_msg():
    server_socket_write(sock, b"Please enter your mission.")
    server_socket_write(sock, b"0: One-by-one move mission")
    server_socket_write(sock, b"1: User-defined item classification move mission")
    server_socket_write(sock, b"2-x: Automation mission-x: 0:DESC 1:ASCE")

def mission_0_msg():
    server_socket_write(sock, b"Please enter your move command.")
    server_socket_write(sock, b"Example: 0-1,2-3 => Plate0->Plate1, position2->position3")
    server_socket_write(sock, b"Enter 'exit' to quit")

def mission_1_msg():
    server_socket_write(sock, b"Please enter your move information.")
    server_socket_write(sock, b"Example: 479561238 => Home to Storage: 4->0, 7->1, 9->2, 5->3, 6->4, 1->5, 2->6, 3->7, 8->8")
    server_socket_write(sock, b"Enter 'exit' to quit")


# plate_control 초기화
pc = plate_control(PLATE_POSITIONS)
# pc.release()

# movel(home, ref=DR_BASE, vel=DR_VEL_L, acc=DR_ACC_L)
# pc.grip()

# sock = server_socket_open(21)
# while True:
#     start_msg()
#     res, rx_data = server_socket_read(sock)
#     if res < 0:
#         if res == -1:
#             server_socket_write(sock, b"Error: Client disconnected")
#         elif res == -2:
#             server_socket_write(sock, b"Error: Socket error occurred")
#         continue
#     rx_msg = rx_data.decode()
#     rx_msg = rx_msg.replace("\r","")
#     rx_msg = rx_msg.replace("\n","")
#     if rx_msg in ['0', '1', '2-0', '2-1']:
#         break
#     else:
#         server_socket_write(sock, b"Error: Invalid input")
#         continue

# if rx_msg == '0':
#     while True:
#         mission_0_msg()
#         res, rx_data = server_socket_read(sock)
#         if res < 0:
#             if res == -1:
#                 server_socket_write(sock, b"Error: Client disconnected")
#             elif res == -2:
#                 server_socket_write(sock, b"Error: Socket error occurred")
#             continue
#         rx_msg = rx_data.decode()
#         rx_msg = rx_msg.replace("\r","")
#         rx_msg = rx_msg.replace("\n","")
#         if rx_msg == 'exit':
#             break
#         try:
#             # 입력 형식: "a-b,x-y" (예: "0-1,2-3" -> 플레이트 0->1, 위치 2->3)
#             plate_part, pos_part = rx_msg.split(',')
#             from_plate, to_plate = map(int, plate_part.split('-'))
#             from_idx, to_idx = map(int, pos_part.split('-'))
#             if (0 <= from_plate < NUM_PLATES and 0 <= to_plate < NUM_PLATES and
#                 0 <= from_idx < 9 and 0 <= to_idx < 9):
#                 server_socket_write(sock, b"Moving, please wait...")
#                 success = pc.move_item(from_plate, from_idx, to_plate, to_idx)
#                 if success:
#                     server_socket_write(sock, b"Move completed")
#                 else:
#                     server_socket_write(sock, b"Error: Move failed (no item or target position occupied)")
#             else:
#                 server_socket_write(sock, b"Error: Invalid plate or position index")
#         except ValueError:
#             server_socket_write(sock, b"Error: Invalid input format (e.g., 0-1,2-3)")
#         continue
# elif rx_msg == '1':
#     while True:
#         mission_1_msg()
#         res, rx_data = server_socket_read(sock)
#         if res < 0:
#             if res == -1:
#                 server_socket_write(sock, b"Error: Client disconnected")
#             elif res == -2:
#                 server_socket_write(sock, b"Error: Socket error occurred")
#             continue
#         rx_msg = rx_data.decode()
#         rx_msg = rx_msg.replace("\r","")
#         rx_msg = rx_msg.replace("\n","")
#         if rx_msg == 'exit':
#             break
#         try:
#             # 입력 형식: 9자리 숫자 문자열 (예: "479561238")
#             if len(rx_msg) == 9 and rx_msg.isdigit() and len(set(rx_msg)) == 9:
#                 server_socket_write(sock, b"Moving, please wait...")
#                 # 9자리 숫자를 홈에서 스토리지(1번 플레이트)로의 매핑으로 해석
#                 for storage_idx, home_idx in enumerate(map(int, rx_msg)):
#                     success = pc.move_item(0, home_idx, 1, storage_idx)
#                     if not success:
#                        # server_socket_write(sock, f"Error: Move from home {home_idx} to storage {storage_idx} failed".encode())
#                         break
#                 else:
#                     server_socket_write(sock, b"Move completed")
#             else:
#                 server_socket_write(sock, b"Error: Enter 9 unique digits (0-8)")
#         except ValueError:
#             server_socket_write(sock, b"Error: Invalid input format")
#         continue
# elif rx_msg == '2-0':
#     server_socket_write(sock, b"Sorting in descending order, please wait...")
#     pc.weight_measurement_manger()  # 무게 측정 후 내림차순 정렬
#     server_socket_write(sock, b"Descending sort completed")
# elif rx_msg == '2-1':
#     server_socket_write(sock, b"Sorting in ascending order, please wait...")
#     pc.weight_measurement_manger(ASCE)  # 무게 측정 후 오름차순 정렬
#     server_socket_write(sock, b"Ascending sort completed")

# server_socket_write(sock, b"Program terminated")
# server_socket_close(sock)
print(set_j(home))
