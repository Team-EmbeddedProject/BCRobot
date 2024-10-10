import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')
        self.declare_parameter('track_width', 0.135)  # 트랙 사이 거리
        self.track_width = self.get_parameter('track_width').value
        
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 각 트랙의 속도를 수신하는 서브스크립션 설정
        self.linear_speed_sub = self.create_subscription(
            Float64, 'linear_speed', self.linear_speed_callback, 10)
        self.angular_speed_sub = self.create_subscription(
            Float64, 'angular_speed', self.angular_speed_callback, 10)
        
        # 트랙 속도 초기화
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.yaw = 0.0
        
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.update_odometry)
        
    def linear_speed_callback(self, msg):
        self.linear_speed = msg.data

    def angular_speed_callback(self, msg):
        # self.angular_speed = math.radians(msg.data)
        self.angular_speed = msg.data

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # 로봇의 위치와 방향 업데이트
        self.pose_x += self.linear_speed * math.cos(self.yaw) * dt / 2
        self.pose_y += self.linear_speed * math.sin(self.yaw) * dt / 2
        self.yaw += self.angular_speed * dt * 3
        while self.yaw > math.pi:
            self.yaw -= 2.0 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2.0 * math.pi

        self.get_logger().info(f"Pose X: {self.pose_x}, Pose Y: {self.pose_y}, Yaw: {self.yaw}")
        
        # 오도메트리 메시지 생성
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        
        odom_msg.pose.pose.position.x = self.pose_x
        odom_msg.pose.pose.position.y = self.pose_y
        odom_msg.pose.pose.position.z = 0.0
        
        odom_quat = self.yaw_to_quaternion(self.yaw)
        odom_msg.pose.pose.orientation.x = odom_quat[0]
        odom_msg.pose.pose.orientation.y = odom_quat[1]
        odom_msg.pose.pose.orientation.z = odom_quat[2]
        odom_msg.pose.pose.orientation.w = odom_quat[3]
        
        odom_msg.twist.twist.linear.x = self.linear_speed
        odom_msg.twist.twist.angular.z = self.angular_speed
        
        # 메시지 퍼블리시
        self.odom_publisher.publish(odom_msg)
        
        # TF 변환 방송
        self.broadcast_tf(odom_msg)
        
        self.last_time = current_time
    
    def broadcast_tf(self, odom_msg):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom_msg.header.stamp
        tf_msg.header.frame_id = odom_msg.header.frame_id
        tf_msg.child_frame_id = odom_msg.child_frame_id
        
        tf_msg.transform.translation.x = odom_msg.pose.pose.position.x
        tf_msg.transform.translation.y = odom_msg.pose.pose.position.y
        tf_msg.transform.translation.z = 0.0
        
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(tf_msg)
    
    def yaw_to_quaternion(self, yaw):
        half_yaw = yaw / 2.0
        return [0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)]

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdometryPublisher()
    try:
        rclpy.spin(odom_publisher)  # 노드 스핀 (실행)
    except KeyboardInterrupt:
        print("Shutting down...")  # 종료 메시지 출력
    finally:
        if rclpy.ok():
            odom_publisher.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
