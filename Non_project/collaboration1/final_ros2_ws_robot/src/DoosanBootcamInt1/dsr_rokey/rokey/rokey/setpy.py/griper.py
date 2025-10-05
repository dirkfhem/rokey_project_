
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
node = rclpy.create_node("qc_management", namespace=ROBOT_ID)

DR_init.__dsr__node = node


from DSR_ROBOT2 import (
    set_digital_output,
    get_digital_input,
    OFF,
    ON,
)

class Griper:
    def __init__(self):
        pass
        


    def grip(self):
        self.release()
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)

    def release(self):
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        
        
    def grip_cup(self):
        self.release_cup()
        set_digital_output(1, OFF)
        set_digital_output(2, ON)

    def release_cup(self):
        set_digital_output(1, ON)
        set_digital_output(2, ON)

    # 수정해야함 제대로 이해 안되서 로직이 안됨. 빈상태로 잡았을 때를 읽어와서 에러 상태로 했으면 좋겠긴함.
    def read_input(self, index):
        return get_digital_input(index)