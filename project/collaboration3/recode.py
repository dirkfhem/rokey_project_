#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import csv
import matplotlib.pyplot as plt
from datetime import datetime
import os

class CmdVelRecorder(Node):
    def __init__(self):
        super().__init__('cmd_vel_recorder')
        
        # control/cmd_vel 구독
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # obj_vel 구독
        self.obj_vel_subscription = self.create_subscription(
            Twist,
            'obj_vel',
            self.obj_vel_callback,
            10)
        
        # /odom 구독
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # 데이터 저장용 리스트
        self.cmd_vel_times = []
        self.cmd_vel_linear_x = []
        self.cmd_vel_angular_z = []
        self.obj_vel_times = []
        self.obj_vel_linear_x = []
        self.obj_vel_angular_z = []
        self.odom_times = []
        self.odom_linear_x = []
        self.odom_angular_z = []
        
        # CSV 파일 설정
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.cmd_vel_csv_file = f'cmd_vel_data_{timestamp}.csv'
        self.obj_vel_csv_file = f'obj_vel_data_{timestamp}.csv'
        self.odom_csv_file = f'odom_data_{timestamp}.csv'
        
        # control/cmd_vel CSV 초기화
        with open(self.cmd_vel_csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'Linear_X', 'Angular_Z'])
        
        # obj_vel CSV 초기화
        with open(self.obj_vel_csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'Linear_X', 'Angular_Z'])
        
        # /odom CSV 초기화
        with open(self.odom_csv_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'Linear_X', 'Angular_Z'])
        
        self.get_logger().info(f'Saving control/cmd_vel data to {self.cmd_vel_csv_file}')
        self.get_logger().info(f'Saving obj_vel data to {self.obj_vel_csv_file}')
        self.get_logger().info(f'Saving odom data to {self.odom_csv_file}')

    def cmd_vel_callback(self, msg):
        # 현재 시간
        current_time = self.get_clock().now().to_msg().sec + \
                      self.get_clock().now().to_msg().nanosec / 1e9
        
        # 데이터 저장
        self.cmd_vel_times.append(current_time)
        self.cmd_vel_linear_x.append(msg.linear.x)
        self.cmd_vel_angular_z.append(msg.angular.z)
        
        # CSV에 데이터 추가
        with open(self.cmd_vel_csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([current_time, msg.linear.x, msg.angular.z])
        
        self.get_logger().info(f'Recorded control/cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

    def obj_vel_callback(self, msg):
        # 현재 시간
        current_time = self.get_clock().now().to_msg().sec + \
                      self.get_clock().now().to_msg().nanosec / 1e9
        
        # 데이터 저장
        self.obj_vel_times.append(current_time)
        self.obj_vel_linear_x.append(msg.linear.x)
        self.obj_vel_angular_z.append(msg.angular.z)
        
        # CSV에 데이터 추가
        with open(self.obj_vel_csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([current_time, msg.linear.x, msg.angular.z])
        
        self.get_logger().info(f'Recorded obj_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

    def odom_callback(self, msg):
        # 현재 시간
        current_time = self.get_clock().now().to_msg().sec + \
                      self.get_clock().now().to_msg().nanosec / 1e9
        
        # 데이터 저장
        self.odom_times.append(current_time)
        self.odom_linear_x.append(msg.twist.twist.linear.x)
        self.odom_angular_z.append(msg.twist.twist.angular.z)
        
        # CSV에 데이터 추가
        with open(self.odom_csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([current_time, msg.twist.twist.linear.x, msg.twist.twist.angular.z])
        
        self.get_logger().info(f'Recorded odom: linear.x={msg.twist.twist.linear.x}, angular.z={msg.twist.twist.angular.z}')

    def plot_data(self):
        # 그래프 생성
        plt.figure(figsize=(12, 12))
        
        # control/cmd_vel 선속도 그래프
        plt.subplot(3, 2, 1)
        plt.plot(self.cmd_vel_times, self.cmd_vel_linear_x, label='Cmd Vel Linear X (m/s)', color='blue')
        plt.xlabel('Time (s)')
        plt.ylabel('Linear Velocity (m/s)')
        plt.title('Control/Cmd_Vel Linear X vs Time')
        plt.grid(True)
        plt.legend()

        # control/cmd_vel 각속도 그래프
        plt.subplot(3, 2, 2)
        plt.plot(self.cmd_vel_times, self.cmd_vel_angular_z, label='Cmd Vel Angular Z (rad/s)', color='red')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (rad/s)')
        plt.title('Control/Cmd_Vel Angular Z vs Time')
        plt.grid(True)
        plt.legend()

        # obj_vel 선속도 그래프
        plt.subplot(3, 2, 3)
        plt.plot(self.obj_vel_times, self.obj_vel_linear_x, label='Obj Vel Linear X (m/s)', color='green')
        plt.xlabel('Time (s)')
        plt.ylabel('Linear Velocity (m/s)')
        plt.title('Obj_Vel Linear X vs Time')
        plt.grid(True)
        plt.legend()

        # obj_vel 각속도 그래프
        plt.subplot(3, 2, 4)
        plt.plot(self.obj_vel_times, self.obj_vel_angular_z, label='Obj Vel Angular Z (rad/s)', color='purple')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (rad/s)')
        plt.title('Obj_Vel Angular Z vs Time')
        plt.grid(True)
        plt.legend()

        # odom 선속도 그래프
        plt.subplot(3, 2, 5)
        plt.plot(self.odom_times, self.odom_linear_x, label='Odom Linear X (m/s)', color='orange')
        plt.xlabel('Time (s)')
        plt.ylabel('Linear Velocity (m/s)')
        plt.title('Odom Linear X vs Time')
        plt.grid(True)
        plt.legend()

        # odom 각속도 그래프
        plt.subplot(3, 2, 6)
        plt.plot(self.odom_times, self.odom_angular_z, label='Odom Angular Z (rad/s)', color='cyan')
        plt.xlabel('Time (s)')
        plt.ylabel('Angular Velocity (rad/s)')
        plt.title('Odom Angular Z vs Time')
        plt.grid(True)
        plt.legend()

        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
        # 프로그램 종료 시 그래프 표시
        node.plot_data()
    
    # 노드 종료
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()