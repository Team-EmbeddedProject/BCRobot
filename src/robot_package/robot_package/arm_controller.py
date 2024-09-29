import sys
sys.path.append('/home/pi/MasterPi/')
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Point
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
from HiwonderSDK.Board import setPWMServoPulse, getPWMServoPulse
import numpy as np

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        self.servos_subscription = self.create_subscription(
            Int32MultiArray,
            'arm_servos_commands',
            self.handle_servo_pulses,
            10
        )
        self.servos_subscription  # prevent unused variable warning

        self.coodinate_subscription = self.create_subscription(
            Point,
            'arm_coordinate_commands',
            self.handle_coordinate_input,
            10
        )
        self.coodinate_subscription # prevent unused variable warning

        self.robot_arm = ArmIK()
        self.arm_init()
        
    def arm_init(self):
        setPWMServoPulse(1, 2000, 1500)
        self.robot_arm.servosMove([560, 2490, 1170, 1500])
    
    def arm_grab(self):
        setPWMServoPulse(1, 1000, 1500)
        time.sleep(1)
        self.robot_arm.servosMove([2200, 1200, 1800, 1500], 2000)
        time.sleep(2)
        setPWMServoPulse(1, 2000, 1500)

    def handle_servo_pulses(self, msg):
        self.get_logger().info(f'Received servo pulses: {msg.data}')
        self.robot_arm.servosMove(msg.data, 1000)
    
    def handle_coordinate_input(self, msg):
        self.get_logger().info(f'Received coordinate data: x:{msg.x} y:{msg.y} z:{msg.z}')
        self.robot_arm.setPitchRangeMoving([msg.x, msg.y, msg.z], 0, -90, 90, 1500)
        # time.sleep(2)
        # self.arm_grab()
        # time.sleep(2)
        # self.arm_init()

def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()

    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        pass
    finally:
        arm_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()