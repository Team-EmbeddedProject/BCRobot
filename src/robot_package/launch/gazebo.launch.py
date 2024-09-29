from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 이름
    package_name = 'robot_package'

    urdf_file_name = 'bcrobot.urdf'
    urdf = os.path.join(get_package_share_directory(package_name), 'urdf', urdf_file_name)

    # 패키지의 공유 디렉토리 경로
    world_file_name = 'simulation_map.world'
    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', world_file_name)

    return LaunchDescription([
        # Gazebo 실행 및 월드 파일 로드 + GazeboRosFactory 플러그인 로드
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # 로봇을 Gazebo에 스폰
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', '-file', urdf, '-entity', 'bcrobot'],
            output='screen'
        )
    ])
