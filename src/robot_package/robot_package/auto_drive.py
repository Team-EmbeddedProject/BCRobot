import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class AutoDrive(Node):
    def __init__(self):
        super().__init__('auto_drive')
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.velocity = Twist()

        # 상태와 타이머 설정
        self.state = 'move_forward'
        self.timer_period = 1.0  # 1초마다 호출
        self.timer = self.create_timer(self.timer_period, self.publish_velocity)
        self.state_duration = 0
        self.total_duration = 0  # 전체 경과 시간을 추적
        self.forward_duration = 2.0

    def move_forward(self):
        self.velocity.linear.x = 0.5
        self.velocity.angular.z = 0.0

        self.vel_publisher.publish(self.velocity)
        self.get_logger().info(f'Publishing velocity: move forward: linear={self.velocity.linear.x}, angular={self.velocity.angular.z}')
    
    def turn_left(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.28

        self.vel_publisher.publish(self.velocity)
        self.get_logger().info(f'Publishing velocity: turn left: linear={self.velocity.linear.x}, angular={self.velocity.angular.z}')
    
    def turn_right(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = -0.28

        self.vel_publisher.publish(self.velocity)
        self.get_logger().info(f'Publishing velocity: turn right: linear={self.velocity.linear.x}, angular={self.velocity.angular.z}')
        
    def stop(self):
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0

        self.vel_publisher.publish(self.velocity)
        self.get_logger().info(f'Stopping: linear={self.velocity.linear.x}, angular={self.velocity.angular.z}')

    def publish_velocity(self):
        
        if self.total_duration >= 7.0:
            self.state = 'stop'

        if self.state == 'move_forward':
            self.move_forward()
            self.state_duration += self.timer_period
            if self.state_duration >= self.forward_duration:
                self.state = 'turn_left'
                if self.forward_duration == 2.0:
                    self.forward_duration = 4.0
                elif self.forward_duration == 4.0:
                    self.forward_duration = 2.0
                self.state_duration = 0

        elif self.state == 'turn_left':
            self.turn_left()
            self.state_duration += self.timer_period
            if self.state_duration >= 1.0:
                self.state = 'move_forward'
                self.state_duration = 0

        elif self.state == 'stop':
            self.stop()
            # 타이머를 더 이상 반복하지 않음
            self.timer.cancel()
        
        self.total_duration += self.timer_period

def main(args=None):
    rclpy.init(args=args)
    auto_drive = AutoDrive()

    try:
        rclpy.spin(auto_drive)  # 이벤트 루프를 시작하여 비동기적으로 노드 실행
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            auto_drive.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
