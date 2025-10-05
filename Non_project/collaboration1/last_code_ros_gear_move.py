# pick and place in 1 method. from pos1 to pos2 @20241104

import rclpy
import DR_init
import time

# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 80, 80

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

OFF, ON = 0, 1



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
    drl_script_stop,
    wait,
    DR_BASE,
    DR_TOOL,
    DR_AXIS_Z,
    amove_periodic,
)

from DR_common2 import posx, posj

set_tool("TCP208mm")
set_tcp("GripperSA_rg2_250509")


System_home = posx(366.42, 3.72, 194.39, 179.82, 179.82, -179.79)

System_gear_A_joint = posj(14.67, 22.32, 79.39, 0.13, 78.29, 14.67)
System_gear_A = posx(497.6, 134.3, 44.67, 2.11, -179.83, 2.48)
System_gear_A_up = posx(497.63, 134.29, 90.0, 100.2, 180.0, 100.58)

System_goal_A_joint = posj(-18.96, 23.2, 78.12, 0.07, 78.84, -18.95)
System_goal_A = posx(493.66, -165.88, 43.44, 2.12, -179.83, 2.5)
System_goal_A_up = posx(493.66, -165.88, 90.0, 2.12, -179.83, 2.5)

System_gear_B_joint = posj(5.02, 13.54, 91.26, 0.09, 75.37, 5.01)
System_gear_B = posx(448.01, 43.25, 44.67, 2.13, -179.83, 2.51)
System_gear_B_up = posx(448.01, 43.25, 90.0, 2.12, -179.83, 2.5)

System_goal_B_joint = posj(-30.01, 22.15, 79.63, 0.04, 78.36, -30.01)
System_goal_B = posx(446.14, -253.88, 43.44, 2.02, -179.83, 2.39)
System_goal_B_up = posx(446.14, -253.88, 90.0, 2.04, -179.83, 2.41)

System_gear_C_joint = posj(4.37, 28.18, 70.7, 0.17, 81.29, 4.39)
System_gear_C = posx(553.64, 46.36, 44.67, 2.08, -179.83, 2.46)
System_gear_C_up = posx(553.64, 46.36, 90.0, 2.1, -179.83, 2.48)

System_goal_C_joint = posj(-25.0, 35.97, 58.26, 0.12, 85.92, -24.95)
System_goal_C = posx(551.18, -252.88, 43.44, 2.26, -179.83, 2.63)
System_goal_C_up = posx(551.18, -252.88, 90.0, 2.29, -179.83, 2.66)

System_gear_center_joint = posj(8.26, 21.11, 81.11, 0.14, 77.96, 8.25)
System_gear_center = posx(499.86, 76.6, 45.65, 2.11, -179.83, 2.48)
System_gear_center_up = posx(499.86, 76.6, 90.0, 2.14, -179.83, 2.51)

System_goal_center_joint = posj(-24.5, 26.43, 72.7, 0.15, 80.87, -24.49)
System_goal_center = posx(497.8, -222.51, 70.0, 69.36, -180.0, 69.73)


DR_VEL_=20
DR_ACC_=20

def grip():
    set_digital_output(1,1)
    wait(1.5)
    set_digital_output(1,0)

def relealse():
    set_digital_output(2,1)
    wait(1.5)
    set_digital_output(2,0)

def get_gear(joint,gear,up):
    movej(joint, vel=DR_VEL_, acc=DR_ACC_)
    movel(gear,ref=DR_BASE,vel=DR_VEL_,acc=DR_ACC_)
    grip()
    movel(up,ref=DR_BASE,vel=DR_VEL_,acc=DR_ACC_)

def goal_gear(joint,goal,up):
    movej(joint, vel=DR_VEL_, acc=DR_ACC_)
    movel(goal,ref=DR_BASE,vel=DR_VEL_,acc=DR_ACC_)
    relealse()
    movel(up,ref=DR_BASE,vel=DR_VEL_,acc=DR_ACC_)

def final_goal(joint,goal):
    movej(joint, vel=DR_VEL_, acc=DR_ACC_)
    movel(goal,ref=DR_BASE,vel=DR_VEL_,acc=DR_ACC_)
    force_ctrl()

def force_ctrl():
    task_compliance_ctrl(stx=[2000, 2000, 100, 20, 20, 20])
    time.sleep(0.1)
    set_desired_force(fd=[0, 0, -15, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0])
    while(True):
        if(check_force_condition(axis=DR_AXIS_Z,max=10,ref=DR_TOOL)):
            amove_periodic(amp=[0.00, 0.00, 0.00, 0.00, 0.00, 15.00], period=[0.00, 0.00, 0.00, 0.00, 0.00, 1.00], atime=2.00, repeat=5, ref=0)
            while(True):
                time.sleep(0.1)
                k= get_current_posx()[0]
                print(k)
                if(k[2]<48):
                    relealse()
                    release_force()
                    release_compliance_ctrl()
                    k[2] += 30
                    movel(k,ref=DR_BASE,vel=DR_VEL_,acc=DR_ACC_)
                    break
            # release_force()
            break
            
    
relealse()
movel(System_home,ref=DR_BASE,vel=DR_VEL_,acc=DR_ACC_)
get_gear(System_gear_A_joint,System_gear_A,System_gear_A_up)
goal_gear(System_goal_A_joint,System_goal_A,System_goal_A_up)
get_gear(System_gear_B_joint,System_gear_B,System_gear_B_up)
goal_gear(System_goal_B_joint,System_goal_B,System_goal_B_up)
get_gear(System_gear_C_joint,System_gear_C,System_gear_C_up)
goal_gear(System_goal_C_joint,System_goal_C,System_goal_C_up)
get_gear(System_gear_center_joint,System_gear_center,System_gear_center_up)
final_goal(System_goal_center_joint,System_goal_center)
movel(System_goal_center,ref=DR_BASE,vel=DR_VEL_,acc=DR_ACC_)
movel(System_home,ref=DR_BASE,vel=DR_VEL_,acc=DR_ACC_)
grip()