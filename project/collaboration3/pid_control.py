#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class TurtleBotPIDController(Node):
    def __init__(self, target_linear=0.0, target_angular=0.0):
        super().__init__('turtlebot_pid_controller')
        
        # ROS2 퍼블리셔와 서브스크라이버 설정
        self.cmd_vel_pub = self.create_publisher(Twist, '/control/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.obj_vel_sub = self.create_subscription(Twist,'/obj_vel',self.obj_vel_callback,10)

        # 목표 속도 설정 (선속도: m/s, 각속도: rad/s)
        self.TARGET_LINEAR = max(0.0, min(0.26*0.8, target_linear))                 # 최대 0.26 m/s (Waffle)
        self.TARGET_ANGULAR = max(-1.82*0.8, min(1.82*0.8, target_angular))         # ±1.82 rad/s (Waffle)

        # PID 제어 상수 (선속도)
        self.LINEAR_KP = 1.2                                                # 비례 게인
        self.LINEAR_KI = 0.05                                               # 적분 게인
        self.LINEAR_KD = 0.1                                                # 미분 게인
        self.MAX_LINEAR_OUTPUT = 0.01                                       # 적분 항 출력 제한
        self.MAX_LINEAR_ACCEL = 0.05                                        # 최대 가속도 (m/s^2)
        self.MAX_LINEAR_DECEL = 0.1                                         # 최대 감속도 (m/s^2)

        # PID 제어 상수 (각속도)
        self.ANGULAR_KP = 1.0                                               # 비례 게인
        self.ANGULAR_KI = 0.02                                              # 적분 게인
        self.ANGULAR_KD = 0.05                                              # 미분 게인
        self.MAX_ANGULAR_OUTPUT = 0.005                                      # 적분 항 출력 제한
        self.MAX_ANGULAR_ACCEL = 0.01                                        # 최대 각가속도 (rad/s^2)
        self.MAX_ANGULAR_DECEL = 0.1                                        # 최대 각감속도 (rad/s^2)

        # 최대 속도 제한 (TurtleBot3 Waffle)
        self.MAX_LINEAR_VEL = 0.26*0.8                          # 최대 선속도 (m/s)
        self.MAX_ANGULAR_VEL = 1.82*0.8                         # 최대 각속도 (rad/s)

        # 현재 상태
        self.odom_linear_vel = 0.0                              # 현재 선속도
        self.odom_angular_vel = 0.0                             # 현재 각속도
        self.cmd_linear = 0.0                                   # 명령 선속도
        self.cmd_angular = 0.0                                  # 명령 각속도

        # PID 변수
        self.linear_integral = 0.0                              # 선속도 적분 항
        self.angular_integral = 0.0                             # 각속도 적분 항
        self.linear_prev_error = 0.0                            # 선속도 이전 오차
        self.angular_prev_error = 0.0                           # 각속도 이전 오차
        self.prev_derivative_linear = 0.0                       # 선속도 이전 미분 항
        self.prev_derivative_angular = 0.0                      # 각속도 이전 미분 항

        # 시간 관련 변수
        self.last_ctrl_time = self.get_clock().now()            # 마지막 제어 시간
        self.last_integral_reset_time = self.get_clock().now()  # 적분 항 리셋 시간

        # 제어 타이머 (0.1초마다 실행)
        self.create_timer(0.1, self.apply_cmd_vel)

    def obj_vel_callback(self,twist_msg):
        obj_twist = Twist()
        obj_twist = twist_msg
        self.TARGET_LINEAR = max(0.0, min(0.26*0.8, obj_twist.linear.x))
        self.TARGET_ANGULAR = max(-1.82*0.8, min(1.82*0.8, obj_twist.angular.z))


    def odom_callback(self, msg):
        """오도메트리 데이터를 처리하여 현재 선속도와 각속도를 업데이트"""
        try:
            self.odom_linear_vel = max(0.0, msg.twist.twist.linear.x)  # 음수 선속도 방지
            self.odom_angular_vel = msg.twist.twist.angular.z
            self.get_logger().info(
                f"오도메트리 - 선속도: {self.odom_linear_vel * 3600:.0f} m/h, "
                f"각속도: {self.odom_angular_vel:.2f} rad/s")
        except Exception as e:  
            self.get_logger().warn(f"오도메트리 콜백 오류: {e}")

    def pid_control(self, error, prev_error, integral, dt, kp, ki, kd, output_limit, max_accel, max_decel, prev_derivative):
        """PID 제어: 안티-와인드업 및 미분 항 필터링 적용"""
        proportional = kp * error  # 비례 항
        derivative = kd * (error - prev_error) / dt  # 미분 항
        alpha = 0.2  # 미분 항에 대한 EMA 필터 상수
        filtered_derivative = alpha * derivative + (1 - alpha) * prev_derivative
        delta_cmd = proportional + integral + filtered_derivative

        # 최대 가속도/감속도 제한
        max_delta = max_decel * dt if error < 0 else max_accel * dt

        # 적분 항 제한 (안티-와인드업)
        if abs(delta_cmd) < output_limit:
            integral += ki * error * dt
            integral = max(-output_limit, min(output_limit, integral))

        # 출력 제한
        if delta_cmd > max_delta:
            integral -= (delta_cmd - max_delta) / kp
            delta_cmd = max_delta
        elif delta_cmd < -max_delta:
            integral -= (delta_cmd + max_delta) / kp
            delta_cmd = -max_delta

        return delta_cmd, error, integral, filtered_derivative

    def apply_cmd_vel(self):
        """선속도와 각속도에 PID 제어 적용"""
        current_time = self.get_clock().now()
        dt = max(0.05, min(0.2, (current_time - self.last_ctrl_time).nanoseconds / 1e9))
        self.last_ctrl_time = current_time

        if (current_time - self.last_integrall_reset_time).nanoseconds / 1e9 >= 3.0:
            self.linear_integral = 0.0
            self.angular_integral = 0.0
            self.prev_derivative_linear = 0.0  # 미분항 초기화 추가
            self.prev_derivative_angular = 0.0  # 미분항 초기화 추가
            self.last_integral_reset_time = current_time
            self.get_logger().info("적분항 및 미분항 초기화")

        twist = Twist()

        # 선속도 PID 제어
        if abs(self.TARGET_LINEAR) < 1e-4:
            self.cmd_linear = 0.0
            self.linear_integral = 0.0
            self.linear_prev_error = 0.0
            self.prev_derivative_linear = 0.0
        else:
            linear_error = self.TARGET_LINEAR - self.odom_linear_vel
            delta_cmd, self.linear_prev_error, self.linear_integral, self.prev_derivative_linear = self.windup_control(
                linear_error, self.linear_prev_error, self.linear_integral, dt,
                self.LINEAR_KP, self.LINEAR_KI, self.LINEAR_KD, self.MAX_LINEAR_OUTPUT,
                self.MAX_LINEAR_ACCEL, self.MAX_LINEAR_DECEL, self.prev_derivative_linear
            )
            self.cmd_linear += delta_cmd
            self.cmd_linear = max(0.0, min(self.MAX_LINEAR_VEL, self.cmd_linear))

        # 각속도 PID 제어
        if abs(self.TARGET_ANGULAR) < 1e-4:
            self.cmd_angular = 0.0
            self.angular_integral = 0.0
            self.angular_prev_error = 0.0
            self.prev_derivative_angular = 0.0
        else:
            angular_error = self.TARGET_ANGULAR - self.odom_angular_vel
            delta_cmd, self.angular_prev_error, self.angular_integral, self.prev_derivative_angular = self.windup_control(
                angular_error, self.angular_prev_error, self.angular_integral, dt,
                self.ANGULAR_KP, self.ANGULAR_KI, self.ANGULAR_KD, self.MAX_ANGULAR_OUTPUT,
                self.MAX_ANGULAR_ACCEL, self.MAX_ANGULAR_DECEL, self.prev_derivative_angular
            )
            self.cmd_angular += delta_cmd
            self.cmd_angular = max(-self.MAX_ANGULAR_VEL, min(self.MAX_ANGULAR_VEL, self.cmd_angular))

        twist.linear.x = self.cmd_linear
        twist.angular.z = self.cmd_angular

        try:
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(
                f"명령 적용 - 목표 선속도: {self.TARGET_LINEAR * 3600:.0f} m/h, "
                f"명령 선속도: {twist.linear.x * 3600:.0f} m/h, "
                f"목표 각속도: {self.TARGET_ANGULAR:.2f} rad/s, "
                f"명령 각속도: {twist.angular.z:.2f} rad/s")
        except Exception as e:
            self.get_logger().warn(f"cmd_vel 퍼블리시 오류: {e}")

    def stop_robot(self):
        """로봇 정지 및 PID 상태 초기화"""
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0
        self.linear_integral = 0.0
        self.angular_integral = 0.0
        self.linear_prev_error = 0.0
        self.angular_prev_error = 0.0
        self.prev_derivative_linear = 0.0
        self.prev_derivative_angular = 0.0
        twist = Twist()
        try:
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("로봇 정지")
        except Exception as e:
            self.get_logger().warn(f"로봇 정지 오류: {e}")

if __name__ == '__main__':
    try:
        rclpy.init()
        # 예시: 선속도 0.1 m/s, 각속도 0.5 rad/s로 설정
        controller = TurtleBotPIDController(target_linear=0.0, target_angular=0.0)
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.stop_robot()
        rclpy.shutdown()
