import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

# 자율주행 시연 런치 파일 - BCRobot
def generate_launch_description():

    # motor_controller 노드 정의
    motor_controller_node = Node(
        package='robot_package',
        executable='motor_controller',
        name='motor_controller',
        output='screen'
    )

    # auto_drive 노드 정의
    auto_drive_node = Node(
        package='robot_package',
        executable='auto_drive',
        name='auto_drive',
        output='screen'
    )

    return LaunchDescription([
        # motor_controller 노드 실행
        motor_controller_node,

        # motor_controller 노드가 시작된 후 auto_drive 노드 실행
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=motor_controller_node,
                on_start=[auto_drive_node],
            )
        )
    ])
