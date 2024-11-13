import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

# 원격제어 시연 런치 파일 - BCRobot
def generate_launch_description():

    # arm_controller 노드 정의
    arm_controller_node = Node(
        package='robot_package',
        executable='arm_controller',
        name='arm_controller',
        output='screen'
    )

    # motor_controller 노드 정의
    motor_controller_node = Node(
        package='robot_package',
        executable='motor_controller',
        name='motor_controller',
        output='screen'
    )

    return LaunchDescription([
        arm_controller_node,
        motor_controller_node
    ])
