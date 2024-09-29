import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file = os.path.join(
        get_package_share_directory('robot_package'),
        'urdf',
        'bcrobot.urdf')

    ydlidar_parameter_file = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'params',
        'ydlidar.yaml'
    )

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false', # 노드간 시간 동기화
            description='Use simulation (Gazebo) clock if true'), 

        # 로봇 상태 퍼블리셔 실행
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description},
                        {'use_sim_time': use_sim_time}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),

        # Odometry 퍼블리셔 실행
        Node(
            package='robot_package',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen'
        ),

        # 라이다 센서 퍼블리셔 실행
        LifecycleNode(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[ydlidar_parameter_file, {'ros__parameters': {'ros__qos': 'keep_all'}}], 
            namespace='/',
        ),

        # 라이다 센서 TF 실행 (base_link -> laser_frame)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_laser',
            arguments=['-0.1', '0', '0.235', '0', '0', '0', '1','base_link','laser_frame'],
        )
    ])
