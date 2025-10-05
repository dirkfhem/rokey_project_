# Base 홈 좌표
System_home = posx(366.42, 3.72, 194.39, 179.82, 179.82, -179.79)

# 가져올 기어 중앙 좌표 (무게중심)
System_gear_center = posx(499.86, 76.6, 45.65, 2.11, -179.83, 2.48)

# 나둘 기어 중앙 좌표 (Y축으로 29cm 이동 반영)
System_goal_center = posx(499.86, 76.6 - 290, 70.0, 69.36, -180.0, 69.73)

DR_ACC_L = 40
DR_VEL_L = 40
DR_ACC_J = 20
DR_VEL_J = 20
DR_BASE = 0    # Reference frame
DR_TOOL = 0    # Tool reference
DR_AXIS_Z = 2  # Z-axis for force condition

def grip():
    set_digital_output(1, 1)
    wait(1.5)
    set_digital_output(1, 0)

def release():
    set_digital_output(2, 1)
    wait(1.5)
    set_digital_output(2, 0)

def set_j(l):
    """
    Convert posx to posj using ikin.
    posx: [x, y, z, rx, ry, rz]
    Returns: [j1, j2, j3, j4, j5, j6] or None if no solution
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

def set_l_up(l):
    """
    Set the position to an 'up' position (Z = 90mm) with same orientation.
    """
    return posx(l[0], l[1], 90.0, l[3], l[4], l[5])

def get_gear(gear):
    j = set_j(set_l_up(gear))
    if j is None:
        return
    movej(j, vel=DR_VEL_J, acc=DR_ACC_J)
    movel(gear, ref=DR_BASE, vel=DR_VEL_L, acc=DR_ACC_L)
    grip()
    movel(set_l_up(gear), ref=DR_BASE, vel=DR_VEL_L, acc=DR_ACC_L)

def goal_gear(goal):
    j = set_j(set_l_up(goal))
    if j is None:
        return
    movej(j, vel=DR_VEL_J, acc=DR_ACC_J)
    movel(goal, ref=DR_BASE, vel=DR_VEL_L, acc=DR_ACC_L)
    release()
    movel(set_l_up(goal), ref=DR_BASE, vel=DR_VEL_L, acc=DR_ACC_L)

def move_ABC(center_gear_posx, center_goal_posx):
    gear_A_posx, gear_B_posx, gear_C_posx = posx_ABC(center_gear_posx)
    goal_A_posx, goal_B_posx, goal_C_posx = posx_ABC(center_goal_posx)
    get_gear(gear_A_posx)
    goal_gear(goal_A_posx)
    get_gear(gear_B_posx)
    goal_gear(goal_B_posx)
    get_gear(gear_C_posx)
    goal_gear(goal_C_posx)

def posx_ABC(center_posx):
    """
    Calculate A, B, C positions based on center_posx (centroid) using geometry.
    center_posx: [x, y, z, rx, ry, rz] (centroid of the triangle)
    Returns: (posx_A, posx_B, posx_C) with distance adjusted to match 6.2cm and 10.4cm
    """
    import math
    x, y, z, rx, ry, rz = center_posx
    # Adjust centroid_to_vertex to match real measurement (6.2cm)
    centroid_to_vertex = 62  # 6.2cm = 62mm
    # Verify side length: s = centroid_to_vertex * sqrt(3) ≈ 107.4mm, adjust to 104mm
    # Recalculate centroid_to_vertex for side length 104mm
    centroid_to_vertex = 104 / math.sqrt(3)  # ~60mm, but use 62mm as per measurement
    angle_step = 2 * math.pi / 3  # 120 degrees for equilateral triangle

    # Calculate A, B, C positions
    posx_A = posx(
        x + centroid_to_vertex * math.cos(0),
        y + centroid_to_vertex * math.sin(0),
        z,
        rx, ry, rz
    )
    posx_B = posx(
        x + centroid_to_vertex * math.cos(angle_step),
        y + centroid_to_vertex * math.sin(angle_step),
        z,
        rx, ry, rz
    )
    posx_C = posx(
        x + centroid_to_vertex * math.cos(2 * angle_step),
        y + centroid_to_vertex * math.sin(2 * angle_step),
        z,
        rx, ry, rz
    )

    return posx_A, posx_B, posx_C

def posx_ABC_from_A(center_posx, point_A):
    """
    Calculate A, B, C positions based on center_posx and point_A.
    center_posx: [x, y, z, rx, ry, rz] (centroid position)
    point_A: [x, y, z, rx, ry, rz] (A position)
    Returns: (posx_A, posx_B, posx_C)
    """
    import math
    center_x, center_y, center_z, center_rx, center_ry, center_rz = center_posx
    a_x, a_y, a_z, a_rx, a_ry, a_rz = point_A

    # Calculate distance from centroid to A
    centroid_to_vertex = math.sqrt((a_x - center_x)**2 + (a_y - center_y)**2)
    # Calculate angle of A relative to center
    angle_A = math.atan2(a_y - center_y, a_x - center_x)
    angle_step = 2 * math.pi / 3  # 120 degrees for equilateral triangle

    # A is already given
    posx_A = point_A
    # Calculate B, C positions
    posx_B = posx(
        center_x + centroid_to_vertex * math.cos(angle_A + angle_step),
        center_y + centroid_to_vertex * math.sin(angle_A + angle_step),
        center_z,
        center_rx, center_ry, center_rz
    )
    posx_C = posx(
        center_x + centroid_to_vertex * math.cos(angle_A + 2 * angle_step),
        center_y + centroid_to_vertex * math.sin(angle_A + 2 * angle_step),
        center_z,
        center_rx, center_ry, center_rz
    )

    return posx_A, posx_B, posx_C

def final_goal(goal):
    posj = set_j(set_l_up(goal))
    if posj is None:
        return
    movej(posj, vel=DR_VEL_J, acc=DR_ACC_J)
    movel(goal, ref=DR_BASE, vel=DR_VEL_L, acc=DR_ACC_L)
    force_ctrl()

def auto_grip():
    
    set_desired_force([0, 0, -15, 0, 0, 0], [0, 0, 1, 0, 0, 0])
    while True:
        if check_force_condition(axis=DR_AXIS_Z, min=10, ref=DR_TOOL):
            release()
            grip()
            release_force()

            


def force_ctrl():
    task_compliance_ctrl([2000, 2000, 100, 20, 20, 20])
    set_desired_force([0, 0, -15, 0, 0, 0], [0, 0, 1, 0, 0, 0])
    while True:
        if check_force_condition(axis=DR_AXIS_Z, min=10, ref=DR_TOOL):
            amove_periodic(amp=[0.00, 0.00, 0.00, 0.00, 0.00, 15.00], period=[0.00, 0.00, 0.00, 0.00, 0.00, 1.00], atime=2.00, repeat=5, ref=0)
            while True:
                k = get_current_posx()[0]
                if k[2] > 45:
                    release()
                    stop(DR_QSTOP)
                    break
            release_force()
            break

# Test with System_gear_center
gear_A, gear_B, gear_C = posx_ABC(System_gear_center)
print(f"gear_A: {gear_A}")
print(f"gear_B: {gear_B}")
print(f"gear_C: {gear_C}")

# Calculate distances
import math
center_x, center_y, center_z = System_gear_center[0], System_gear_center[1], System_gear_center[2]
a_x, a_y, a_z = gear_A[0], gear_A[1], gear_A[2]
b_x, b_y, b_z = gear_B[0], gear_B[1], gear_B[2]
c_x, c_y, c_z = gear_C[0], gear_C[1], gear_C[2]

dist_center_to_a = math.sqrt((a_x - center_x)**2 + (a_y - center_y)**2 + (a_z - center_z)**2)
dist_center_to_b = math.sqrt((b_x - center_x)**2 + (b_y - center_y)**2 + (b_z - center_z)**2)
dist_center_to_c = math.sqrt((c_x - center_x)**2 + (c_y - center_y)**2 + (c_z - center_z)**2)
dist_a_to_b = math.sqrt((b_x - a_x)**2 + (b_y - a_y)**2 + (b_z - a_z)**2)
dist_b_to_c = math.sqrt((c_x - b_x)**2 + (c_y - b_y)**2 + (c_z - b_z)**2)
dist_c_to_a = math.sqrt((a_x - c_x)**2 + (a_y - c_y)**2 + (a_z - c_z)**2)

print(f"Center to A: {dist_center_to_a} mm")
print(f"Center to B: {dist_center_to_b} mm")
print(f"Center to C: {dist_center_to_c} mm")
print(f"A to B: {dist_a_to_b} mm")
print(f"B to C: {dist_b_to_c} mm")
print(f"C to A: {dist_c_to_a} mm")

# Main execution
release()
movel(System_home, ref=DR_BASE, vel=DR_VEL_L, acc=DR_ACC_L)
move_ABC(System_gear_center, System_goal_center)
get_gear(System_gear_center)
final_goal(System_goal_center)
movel(System_home, ref=DR_BASE, vel=DR_VEL_L, acc=DR_ACC_L)
