import rclpy
import DR_init
import time
import numpy as np
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
OFF, ON = 0, 1         
VELOCITY, ACC = 80, 80
DR_VEL_=20
DR_ACC_=20
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("pjt_test", namespace=ROBOT_ID)

    DR_init.__dsr__node = node
    

    try:
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
                drl_script_stop,
                wait,
                DR_BASE,
                DR_TOOL,
                DR_AXIS_Z,
                amove_periodic,
                trans,
        )
        set_tool("TCP208mm")
        set_tcp("GripperSA_rg2_250509")


        def set_j( pos_x, solspace=2):
            """pos_x를 입력받아 pos_j(관절 좌표)를 계산"""
            try:
                j = ikin(pos_x, solspace)
                return j.tolist() if isinstance(j, np.ndarray) else j
            except Exception as e:
                print(f"ikin error: {e}")
                return None
        def grip15():   
            set_digital_output(1,0)
            wait(0.2)
            set_digital_output(2,0)
        def release50():
            set_digital_output(1,1)
            wait(0.2)
            set_digital_output(2,0)
        def release75():
            set_digital_output(1,0)
            wait(0.2)
            set_digital_output(2,1)
        def release95():
            set_digital_output(1,1)
            wait(0.2)
            set_digital_output(2,1)


        def force_ctrl():
            grip15()
            time.sleep(0.5)
            task_compliance_ctrl(stx=[20000, 20000, 1000, 20, 20, 20])
            time.sleep(0.1)
            set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
            while(True):
                if(check_force_condition(axis=DR_AXIS_Z,max=10,ref=DR_TOOL)):
                    print('밀착 성공')
                    now = get_current_posx()[0]
                    now[2]+=5
                    time.sleep(0.1)
                    release_force()
                    release_compliance_ctrl()
                    print('힘 해제')
                    time.sleep(0.5)
                    movel(now,vel=VELOCITY, acc=ACC)
                    print('살짝 올라가기')
                    release75()
                    print('벌리기')
                    time.sleep(0.5)
                    print('다시 누르기')
                    task_compliance_ctrl(stx=[20000, 20000, 1000, 20, 20, 20])
                    time.sleep(0.1)
                    set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
                    time.sleep(1.9)  # 벌리고 내려가기 1초동안
                    print('잡기')
                    grip15()
                    time.sleep(0.1)
                    print('힘 해제')
                    release_force()
                    release_compliance_ctrl()
                    time.sleep(0.2)
                    print('윗방향 힘')
                    task_compliance_ctrl(stx=[20000, 20000, 1000, 20, 20, 20])
                    set_desired_force(fd=[0, 0, 10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
                    amove_periodic(amp = [0,0,0,0,5,0],period=[0,0,0,0,3,0], atime=0.2, repeat=3, ref=DR_TOOL)
                    time.sleep(3)
                    release_force()
                    release_compliance_ctrl()
                    print('끝')
                    break

        def force_ctrl2():
            task_compliance_ctrl(stx=[20000, 20000, 1000, 20, 20, 20])
            time.sleep(0.1)
            set_desired_force(fd=[0, 0, -30, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
            amove_periodic(amp = [0,0,0,0,3,0],period=[0,0,0,0,3,0], atime=0.2, repeat=3, ref=DR_TOOL)
            time.sleep(3)
            while(True):
                if(check_force_condition(axis=DR_AXIS_Z,max=10,ref=DR_TOOL)):
                    print('밀착 성공')
                    time.sleep(0.1)
                    release_force()
                    release_compliance_ctrl()
                    print('힘 해제')
                    time.sleep(0.5)
                    
                    release75()
                    time.sleep(1.0)
                    break
        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    JReady = [0, 0, 90, 0, 90, 0]

    J1 = [-8.30, 46.19, 48.60, 0.26, 85.26, -8.21]
    J2 = [-4.15, 45.30, 50.21, 0.27, 84.55, -4.07]
    J3 =[-0.66, 45.05, 50.67, 0.27, 84.34, -0.58]
    J4 =  [2.14, 45.16, 50.46, 0.27, 84.43, 2.22]
    J5 =[4.92, 45.56, 49.75, 0.28, 84.75, 5.00]
    J6 =[7.68, 46.24, 48.52, 0.28, 85.30, 7.77]
    J7 = [10.40, 47.22, 46.73, 0.29, 86.10, 10.49]
    J8 = [13.07, 48.53, 44.33, 0.29, 87.18, 13.17]

    Jg1 = [-12.84, 12.60, 100.98, 0.09, 66.48, -12.87]
    Jg2 =[-6.42, 11.61, 102.22, 0.09, 66.23, -6.45]
    Jg3 =[-0.94, 11.32, 102.57, 0.09, 66.15, -0.97]
    Jg4 =[2.79, 15.32, 97.46, 0.12, 67.27, -87.24]
    Jg5 =[-1.29, 19.26, 92.12, 0.14, 68.68, -91.32]
    Jg6 =[-5.35, 15.44, 97.31, 0.11, 67.31, -95.38]
    Jg7 =[-8.77, 20.04, 91.02, 0.14, 69.00, -98.0]
    Jg8 =[-11.74, 24.77, 84.15, 0.16, 71.14, -101.76]

    P1 = [655.52, -91.02, 38.30, 161.52, 179.94, 161.90]
    
    Pg1 = [415.84, -90.77, 38.33, 160.90, 179.94, 161.28]
    P2 = posx(655.53, -43.03, 38.31, 161.43, 179.94, 161.81)
    Pg2 = posx(415.81, -2.77, 38.31, 161.95, 179.94, 162.38)
    P3 = posx(655.53, -3.03, 38.33, 161.05, 179.94, 161.43)
    Pg3 = posx(445.84, 25.77, 38.33, 161.07, 179.94, 71.45)
    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    while rclpy.ok():
        print("movej")
        movej(JReady, vel=VELOCITY, acc=ACC)


        # task_compliance_ctrl(stx=[20000, 20000, 100, 20, 20, 20])
        # set_desired_force(fd=[0, 0, 10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
        # amove_periodic(amp = [0,0,0,0,7,0],period=[0,0,0,0,2.5,0], atime=0.2, repeat=3, ref=DR_TOOL)
        # time.sleep(10)
        # release_force()
        # release_compliance_ctrl()
        
        #1
        movej(J1,vel=VELOCITY, acc=ACC)
        force_ctrl()
        movej(J1,vel=VELOCITY, acc=ACC)
        # print('올라가기1')
        # time.sleep(1)
        # movel(Pred_up,vel=VELOCITY, acc=ACC)
        print('타겟이동1')
        # movel(Pgrn_up, vel=90, acc=90)
        movej(Jg1, vel=VELOCITY, acc=ACC)
        print('내려가요')
        force_ctrl2()
        movej(Jg1, vel=VELOCITY, acc=ACC)
        
        #2
        movej(J2,vel = VELOCITY, acc =ACC)
        force_ctrl()
        movej(J2,vel = VELOCITY, acc =ACC)
        movej(Jg2,vel = VELOCITY, acc =ACC)
        force_ctrl2()
        movej(Jg2,vel = VELOCITY, acc =ACC)

        #3
        movej(J3,vel = VELOCITY, acc =ACC)
        force_ctrl()
        movej(J3,vel = VELOCITY, acc =ACC)
        movej(Jg3,vel = VELOCITY, acc =ACC)
        force_ctrl2()
        movej(Jg3,vel = VELOCITY, acc =ACC)

        #4
        movej(J4,vel = VELOCITY, acc =ACC)
        force_ctrl()
        movej(J4,vel = VELOCITY, acc =ACC)
        movej(Jg4,vel = VELOCITY, acc =ACC)
        force_ctrl2()
        movej(Jg4,vel = VELOCITY, acc =ACC)

        #5
        movej(J5,vel = VELOCITY, acc =ACC)
        force_ctrl()
        movej(J5,vel = VELOCITY, acc =ACC)
        movej(Jg5,vel = VELOCITY, acc =ACC)
        force_ctrl2()
        movej(Jg5,vel = VELOCITY, acc =ACC)

        #6
        movej(J6,vel = VELOCITY, acc =ACC)
        force_ctrl()
        movej(J6,vel = VELOCITY, acc =ACC)
        movej(Jg6,vel = VELOCITY, acc =ACC)
        force_ctrl2()
        movej(Jg6,vel = VELOCITY, acc =ACC)

        #7
        movej(J7,vel = VELOCITY, acc =ACC)
        force_ctrl()
        movej(J7,vel = VELOCITY, acc =ACC)
        movej(Jg7,vel = VELOCITY, acc =ACC)
        force_ctrl2()
        movej(Jg7,vel = VELOCITY, acc =ACC)

        #8
        movej(J8,vel = VELOCITY, acc =ACC)
        force_ctrl()
        movej(J8,vel = VELOCITY, acc =ACC)
        movej(Jg8,vel = VELOCITY, acc =ACC)
        force_ctrl2()
        movej(Jg8,vel = VELOCITY, acc =ACC)









        print('집가요')
        movej(JReady, vel=VELOCITY, acc=ACC)
        break
    rclpy.shutdown()

if __name__ == "__main__":

    main()

