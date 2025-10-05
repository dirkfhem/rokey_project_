#!/usr/bin/env python3
#
# Copyright 2025 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Based on control_lane.launch.py
# Purpose: Launch file for traffic light control module

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Traffic light control module launch description"""
    
    control_traffic_node = Node(
        package='turtlebot3_autorace_mission',
        executable='control_traffic',
        name='control_traffic',
        output='screen'
        # 모든 토픽이 기본값으로 올바르게 설정되어 있으므로 remapping 불필요
        # 입력: /control/cmd_vel (control_lane에서)
        # 입력: /detect/traffic_light (detect_traffic_light에서)
        # 입력: /detect/*_reliability (detect_traffic_light에서)
        # 출력: /cmd_vel (로봇으로)
        # 출력: /traffic_light_override (다른 노드 참조용)
    )
    
    return LaunchDescription([
        control_traffic_node
    ])