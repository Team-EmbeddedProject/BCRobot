import sys
sys.path.append('/home/pi/MasterPi/')
import time
from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
from HiwonderSDK.Board import setPWMServoPulse
import numpy as np

class Mode(Enum):
    AUTO = 100
    REMOTE = 200

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        self.mode = Mode.REMOTE  # ì´ˆê¸°?Š” ?ž?™ ëª¨ë“œ
        self.mode_subscriber = self.create_subscription(Int16, 'mode', self.robot_mode_callback, 10)

        self.coordinate_subscription = self.create_subscription(
            Point,
            'arm_coordinate_commands',
            self.handle_coordinate_input,
            10
        )
        self.coordinate_subscription  # prevent unused variable warning

        self.robot_arm = ArmIK()
        if self.mode == Mode.REMOTE:
            self.arm_front_view()
        elif self.mode == Mode.AUTO:
            self.arm_init()
        self.executed_once = False  # ?™?ž‘?´ ?•œ ë²? ?‹¤?–‰?˜?—ˆ?Š”ì§? ?™•?¸?•˜?Š” ë³??ˆ˜ ì¶”ê??
        
    def arm_init(self):
        setPWMServoPulse(1, 2000, 1500)
        self.robot_arm.servosMove([700, 2400, 1350, 1500], 2000)
        self.get_logger().info("Arm initialized to default position.")
        
    def arm_grab(self):
        setPWMServoPulse(1, 1000, 1000)
        time.sleep(3)
        self.robot_arm.servosMove([1800, 700, 1700, 1500], 1500)
        time.sleep(5)
        setPWMServoPulse(1, 2000, 1000)

    def arm_front_view(self):
        setPWMServoPulse(1, 2000, 1500)
        self.robot_arm.servosMove([700, 2200, 1000, 1500], 1500)
        self.get_logger().info("Moving robot arm to front view position for camera alignment.")

    def robot_mode_callback(self, msg):
        self.get_logger().info(f'Received mode: {msg.data}')
        try:
            self.mode = Mode(msg.data)  
            if self.mode == Mode.REMOTE:
                self.arm_front_view()
            elif self.mode == Mode.AUTO:
                self.arm_init()
        except ValueError:
            self.get_logger().error(f"Invalid mode received: {msg.data}")

    def handle_coordinate_input(self, msg):
        if self.executed_once:  # ?™?ž‘?´ ?´ë¯? ?‹¤?–‰?˜?—ˆ?‹¤ë©? ?” ?´?ƒ ì²˜ë¦¬?•˜ì§? ?•Š?Œ
            return

        self.get_logger().info(f"Receiving subscription!")
        self.executed_once = True  # ?™?ž‘?´ ?•œ ë²? ?‹¤?–‰?˜?—ˆ?Œ?„ ê¸°ë¡

        if self.mode == Mode.AUTO:
            try:
                x = msg.x  # mm 
                y = msg.y 
                z = msg.z

                self.get_logger().info(f"Received coordinate data: x={x} mm, y={y} mm, z={z} mm")

                if not self.is_within_workspace(x, y, z):
                    self.get_logger().warn("Target coordinates are out of the robot arm's workspace.")
                    return

                self.robot_arm.servosMove([1200, 2300, 2000, 1500], 1500)
                time.sleep(4)

                self.arm_grab()
                time.sleep(4)
                self.arm_init()

                # ?•œ ë²? ?™?ž‘ ?›„ ?…¸?“œ ì¢…ë£Œ
                self.get_logger().info("Shutting down after first successful arm operation.")
                rclpy.shutdown()  # ROS ?…¸?“œ ì¢…ë£Œ
            
            except Exception as e:
                self.get_logger().error(f"Error in handle_coordinate_input: {e}")

    def is_within_workspace(self, x, y, z):
        workspace_radius = 1000  # mm
        distance = np.sqrt(x**2 + y**2)
        if distance > workspace_radius or z < 0 or z > 1000:
            return False
        return True

def main(args=None):
    rclpy.init(args=args)
    arm_controller = ArmController()

    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            arm_controller.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
