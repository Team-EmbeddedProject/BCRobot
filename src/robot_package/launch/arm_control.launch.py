import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

# 쓰레기 인식 및 수거 시연 런치 파일 - BCRobot
def generate_launch_description():

    # sensor_processing 노드 정의 -> BCRobot 센서 값 퍼블리시
    sensor_processing_node = Node(
        package='robot_package',
        executable='sensor_processing',
        name='sensor_processing',
        output='screen'
    )

    # arm_controller 노드 정의 -> BCRobot 로봇 팔 제어
    arm_controller_node = Node(
        package='robot_package',
        executable='arm_controller',
        name='arm_controller',
        output='screen'
    )

    return LaunchDescription([
        # sensor_processing 노드 실행
        sensor_processing_node,
        arm_controller_node
    ])
