import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 디렉토리 경로 가져오기
    package_name = 'robot_package'
    urdf_file_name = 'bcrobot.urdf'
    
    urdf_file = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file_name)

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        # 로봇 상태 퍼블리셔 실행
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        )
    ])
