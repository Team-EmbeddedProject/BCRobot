import sys
sys.path.append('/home/pi/MasterPi/')
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import HiwonderSDK.mecanum as mecanum

class KeyboardMotorController(Node):
    def __init__(self):
        super().__init__('keyboard_motor_controller')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )
        self.chassis = mecanum.MecanumChassis()

        # 속도 제한 설정
        self.max_velocity = 50  # 최대 속도 (단위: cm/s)
        self.max_angular = 0.5   # 최대 각속도 (단위: rad/s)

        self.linear_speed_pub = self.create_publisher(Float64, 'linear_speed', 10)
        self.angular_speed_pub = self.create_publisher(Float64, 'angular_speed', 10)

    def listener_callback(self, msg):
        velocity = max(min(msg.linear.x * 100, self.max_velocity), -self.max_velocity)
        angular_rate = max(min(msg.angular.z, self.max_angular), -self.max_angular)

        self.chassis.set_velocity(int(velocity), 90, -angular_rate)
        self.get_logger().info(f'Velocity: {velocity}, Angualr: {angular_rate}')

        velocity_msg = Float64()
        velocity_msg.data = velocity / 100
        self.linear_speed_pub.publish(velocity_msg)

        angular_msg = Float64()
        angular_msg.data = angular_rate
        self.angular_speed_pub.publish(angular_msg)

    def cleanup(self):
        self.chassis.set_velocity(0, 0, 0)
        self.get_logger().info('Node is shutting down, stopping the robot')

def main(args = None):
    rclpy.init(args=args)
    keyboard_motor_controller = KeyboardMotorController()

    try:
        rclpy.spin(keyboard_motor_controller)
    except KeyboardInterrupt:
        keyboard_motor_controller.get_logger().info('Ctrl+C detected, shutting down node.')
    finally:
        if rclpy.ok():
            keyboard_motor_controller.cleanup()
            keyboard_motor_controller.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
        