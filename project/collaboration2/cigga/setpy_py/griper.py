#그리퍼 모듈 불러오기
import time
from onrobot import RG
GRIPPER_NAME = 'rg2'
TOOLCHARGER_IP = '192.168.1.1'
TOOLCHARGER_PORT = '502'
GRIPPER_FORCE=100
gripper = RG(GRIPPER_NAME,TOOLCHARGER_IP,TOOLCHARGER_PORT)

class Griper:
    def __init__(self):
        self.gripper = RG(GRIPPER_NAME,TOOLCHARGER_IP,TOOLCHARGER_PORT)
    
    def grip(self,force=GRIPPER_FORCE):
        gripper.move_gripper(0,force)
        while gripper.get_status()[0]:
            time.sleep(0.2)

    def release(self,size,force=GRIPPER_FORCE):
        gripper.move_gripper(size+100,force)
        while gripper.get_status()[0]:
            time.sleep(0.2)

    # 수정해야함 제대로 이해 안되서 로직이 안됨. 빈상태로 잡았을 때를 읽어와서 에러 상태로 했으면 좋겠긴함.
    def read_with(self):
        return gripper.get_status[1],gripper.get_width_with_offset()

def main():
    gr = Griper()
    gr.grip(500,400)
    gr.release(500,400)
    print(gr.read_with())