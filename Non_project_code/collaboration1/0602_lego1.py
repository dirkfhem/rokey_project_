import rclpy
import DR_init
import time
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
OFF, ON = 0, 1         
VELOCITY, ACC = 40, 40
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
                    amove_periodic(amp = [0,0,0,0,7,0],period=[0,0,0,0,3,0], atime=0.2, repeat=3, ref=DR_TOOL)
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
                    break
        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    JReady = [0, 0, 90, 0, 90, 0]
    Jgrn = [13.30, 16.97, 94.68, 0.19, 68.50, 13.25]
    Jred = [-10.09, 33.38, 70.12, 0.20, 76.67, -10.07]
    Pred= posx(574.21, -98.11, 270.37, 167.08, 179.83, 167.47)
    Pred_up= posx(574.21, -98.11, 150.37, 167.08, 179.83, 167.47) #z +30
    Pgrn_up = posx(447.38, 109.90, 150, 167.13, 179.83, 167.52)
    Pgrn = posx(447.38, 109.90, 270.36, 167.13, 179.83, 167.52)
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
        

        movej(Jred,vel=VELOCITY, acc=ACC)
        force_ctrl()
        movej(Jred,vel=VELOCITY, acc=ACC)
        print('올라가기')
        time.sleep(1)
        movel(Pred_up,vel=VELOCITY, acc=ACC)
        
        movel(Pgrn_up, vel=80, acc=80)
        movej(Jgrn, vel=VELOCITY, acc=ACC)
        print('내려가요')
        force_ctrl2()
        movel(Pgrn_up, vel=VELOCITY, acc=ACC)
        print('집가요')
        movej(JReady, vel=VELOCITY, acc=ACC)
        break
    rclpy.shutdown()

if __name__ == "__main__":

    main()

