#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt
import glob
import os

# CSV 파일 이름 패턴 설정 (타임스탬프 제외한 접두사)
CMD_VEL_PREFIX = 'cmd_vel_data_'
OBJ_VEL_PREFIX = 'obj_vel_data_'
ODOM_PREFIX = 'odom_data_'
TIME='20250703_110824'
def read_csv_file(file_path):
    """CSV 파일에서 타임스탬프, linear.x, angular.z 데이터를 읽음"""
    times = []
    linear_x = []
    angular_z = []
    
    try:
        with open(file_path, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)  # 헤더 스킵
            for row in reader:
                times.append(float(row[0]))
                linear_x.append(float(row[1]))
                angular_z.append(float(row[2]))
        return times, linear_x, angular_z
    except FileNotFoundError:
        print(f"파일을 찾을 수 없습니다: {file_path}")
        return [], [], []

def plot_csv_data(cmd_vel_file=None, obj_vel_file=None, odom_file=None):
    """CSV 데이터를 읽고 그래프로 시각화"""
    # 데이터 초기화
    cmd_vel_times, cmd_vel_linear_x, cmd_vel_angular_z = [], [], []
    obj_vel_times, obj_vel_linear_x, obj_vel_angular_z = [], [], []
    odom_times, odom_linear_x, odom_angular_z = [], [], []
    
    # CSV 파일 읽기
    if cmd_vel_file:
        cmd_vel_times, cmd_vel_linear_x, cmd_vel_angular_z = read_csv_file(cmd_vel_file)
    if obj_vel_file:
        obj_vel_times, obj_vel_linear_x, obj_vel_angular_z = read_csv_file(obj_vel_file)
    if odom_file:
        odom_times, odom_linear_x, odom_angular_z = read_csv_file(odom_file)
    
    # 그래프 생성
    plt.figure(figsize=(12, 12))
    
    # control/cmd_vel 선속도 그래프
    plt.subplot(3, 2, 1)
    if cmd_vel_times:
        plt.plot(cmd_vel_times, cmd_vel_linear_x, label='Cmd Vel Linear X (m/s)', color='blue')
    plt.xlabel('Time (s)')
    plt.ylabel('Linear Velocity (m/s)')
    plt.title('Control/Cmd_Vel Linear X vs Time')
    plt.grid(True)
    plt.legend()

    # control/cmd_vel 각속도 그래프
    plt.subplot(3, 2, 2)
    if cmd_vel_times:
        plt.plot(cmd_vel_times, cmd_vel_angular_z, label='Cmd Vel Angular Z (rad/s)', color='red')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('Control/Cmd_Vel Angular Z vs Time')
    plt.grid(True)
    plt.legend()

    # obj_vel 선속도 그래프
    plt.subplot(3, 2, 3)
    if obj_vel_times:
        plt.plot(obj_vel_times, obj_vel_linear_x, label='Obj Vel Linear X (m/s)', color='green')
    plt.xlabel('Time (s)')
    plt.ylabel('Linear Velocity (m/s)')
    plt.title('Obj_Vel Linear X vs Time')
    plt.grid(True)
    plt.legend()

    # obj_vel 각속도 그래프
    plt.subplot(3, 2, 4)
    if obj_vel_times:
        plt.plot(obj_vel_times, obj_vel_angular_z, label='Obj Vel Angular Z (rad/s)', color='purple')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('Obj_Vel Angular Z vs Time')
    plt.grid(True)
    plt.legend()

    # odom 선속도 그래프
    plt.subplot(3, 2, 5)
    if odom_times:
        plt.plot(odom_times, odom_linear_x, label='Odom Linear X (m/s)', color='orange')
    plt.xlabel('Time (s)')
    plt.ylabel('Linear Velocity (m/s)')
    plt.title('Odom Linear X vs Time')
    plt.grid(True)
    plt.legend()

    # odom 각속도 그래프
    plt.subplot(3, 2, 6)
    if odom_times:
        plt.plot(odom_times, odom_angular_z, label='Odom Angular Z (rad/s)', color='cyan')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity (rad/s)')
    plt.title('Odom Angular Z vs Time')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

def main():
    # CSV 파일 이름 패턴으로 최신 파일 검색
    cmd_vel_files = glob.glob(f'{CMD_VEL_PREFIX}{TIME}.csv')
    obj_vel_files = glob.glob(f'{OBJ_VEL_PREFIX}{TIME}.csv')
    odom_files = glob.glob(f'{ODOM_PREFIX}{TIME}.csv')

    # 가장 최신 파일 선택 (파일 생성 시간 기준)
    cmd_vel_file = max(cmd_vel_files, key=os.path.getctime) if cmd_vel_files else None
    obj_vel_file = max(obj_vel_files, key=os.path.getctime) if obj_vel_files else None
    odom_file = max(odom_files, key=os.path.getctime) if odom_files else None

    if not (cmd_vel_file or obj_vel_file or odom_file):
        print("CSV 파일을 찾을 수 없습니다.")
        return

    print(f"Reading cmd_vel: {cmd_vel_file or 'None'}")
    print(f"Reading obj_vel: {obj_vel_file or 'None'}")
    print(f"Reading odom: {odom_file or 'None'}")

    # 데이터 플롯
    plot_csv_data(cmd_vel_file, obj_vel_file, odom_file)

if __name__ == '__main__':
    main()