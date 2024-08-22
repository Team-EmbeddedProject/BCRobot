from enum import Enum
import rclpy as rp
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from std_srvs.srv import Trigger
from interface_package.srv import NodeNum
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf_transformations import quaternion_from_euler, euler_from_quaternion

import math
from time import sleep
from collections import namedtuple

MAX_ATTEMPTS = 3
NAVIGATION_TIME_FACTOR = 13
MIN_NAV_TIME = 5.0

Position = namedtuple('Position', ['x', 'y'])

class RobotStatus(Enum):
    HOME = 0  # waiting at Home
    DRIVING = 1
    DETECT_TRASH = 2
    DETECT_OBSTACLE = 3
    RETURNING = 4
    AT_HOME = 5 # arriving at Home

class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")

        self.nav = BasicNavigator()
        self.nav.waitUntilNav2Active()

        self.initialize_parameters()
        self.cmd_vel_pub = self.create_publisher(Twist, "/base_controller/cmd_vel_unstamped", 10)
        self.reset_sub = self.create_service(Trigger, "/reset", self.reset_callback)
        self.robot_arrival_client = self.create_client(NodeNum, "robotArrival")
        self.moving_timer = self.create_timer(1.0, self.moving_timer_callback)
    
        self.get_logger().info(f"OCRobot Motor Controller Start")
        
    def initialize_parameters(self):
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.declare_parameter("points", "")

        self.points_str = self.get_parameter("points").get_parameter_value().string_value
        self.positions, self.labels = self.parse_points(self.points_str)

        self.valid_points = self.get_indices_for_label("Valid")
        self.invalid_points = self.get_indices_for_label("Invalid")
        self.home_point = self.get_indices_for_label("Home")

        self.initialize_variables()

    def initialize_variables(self):
        self.is_active = False
        self.is_moving = False
        self.status = RobotStatus.HOME

        self.current_point = self.home_point[0] # tuple
        self.next_point = self.home_point[0] # tuple
        self.current_position = self.get_position(self.home_point[0]) # namedtuple
        self.next_position = self.get_position(self.home_point[0]) # namedtuple

        self.current_yaw = 0.0
        self.diff_dist = 0.0
        self.attempt_count = 0
        self.max_attempts = 3

        self.x_shift = 0.0
        self.y_shift = 0.0

    def get_position(self, point):
        row, col = point
        return self.positions[row][col]

    def parse_points(self, points_str):
        points = points_str.strip().split(";")
        positions = [[None for _ in range(9)] for _ in range(5)]
        labels = [[None for _ in range(9)] for _ in range(5)]

        for point in points:
            if point:
                key, value = point.split(": ")
                row, col = map(int, key.split(","))
                label, x, y = value.split(", ")
                positions[row][col] = Position(float(x), float(y))
                labels[row][col] = label

        return positions, labels
    
    def get_indices_for_label(self, label_prefix):
        indices = []
        for row in range(5):
            for col in range(9):
                if self.labels[row][col].startswith(label_prefix):
                    indices.append((row, col))
        return indices
    
    def update_positions(self, current_point, next_point):
        self.current_point = current_point
        self.next_point = next_point
        self.current_position = self.get_position(current_point)
        self.next_position = self.get_position(next_point)

    def update_status(self, next_status):
        self.get_logger().info(f"Updating status from {self.status.name}")
        
        self.status = next_status
        self.get_logger().info(f"Status updated to {self.status.name}")

        if self.status == RobotStatus.RETURNING:
            self.returning()
            self.get_logger().info("Returning to Home")
        
        if self.status == RobotStatus.AT_HOME:
            self.reset()
            sleep(1)
            self.status = RobotStatus.HOME
            self.get_logger().info(f"Status updated to {self.status.name}")

    def moving_timer_callback(self):
        if self.is_moving:
            self.get_logger().info(f"move to {self.next_position}")
            self.is_moving = False
            self.send_goal(self.next_position)

    def send_goal(self, goal):
        x = goal.x
        y = goal.y
        goal_pose = self.get_goal_pose(x, y)
        
        distance = self.calc_diff(self.next_position, self.current_position)
        nav_time = max(distance * NAVIGATION_TIME_FACTOR, MIN_NAV_TIME)
    
        self.nav.goToPose(goal_pose)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            if feedback and Duration.from_msg(feedback.navigation_time) > Duration(seconds=nav_time):
                self.nav.cancelTask()
                break

        result = self.nav.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Goal succeeded!")
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("Goal was canceled!")
        elif result == TaskResult.FAILED:
            self.get_logger().warn("Goal failed!")

        if self.check_succeed(self.current_position):
            self.request_robot_arrival(self.next_point)

    def determine_direction(self, current, next):
        dx = next[0] - current[0]
        dy = next[1] - current[1]
        self.get_logger().info(f"Determine direction: current={current}, next={next}, dx={dx}, dy={dy}, status={self.status}")

        if dx > 0 and dy == 0:
            yaw = 0.0
            self.x_shift == 0.0
            self.y_shift == 0.0
        elif dx < 0 and dy == 0:
            yaw = math.pi
            self.x_shift == -0.1
            self.y_shift == 0.0
        elif dx == 0 and dy > 0:
            yaw = math.pi / 2
            self.x_shift == 0.0
            self.y_shift == -0.05
        elif dx == 0 and dy < 0:
            yaw = -math.pi / 2
            self.x_shift == 0.0
            self.y_shift == 0.05
        else:
            yaw = math.atan2(dy, dx)

        return yaw

    def get_goal_pose(self, x, y):
        yaw = self.determine_direction(self.current_point, self.next_point)
        self.get_logger().info(f"Goal yaw : {yaw}")

        orientation_val = quaternion_from_euler(0, 0, yaw)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = x + self.x_shift
        goal_pose.pose.position.y = y + self.y_shift
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = orientation_val[0]
        goal_pose.pose.orientation.y = orientation_val[1]
        goal_pose.pose.orientation.z = orientation_val[2]
        goal_pose.pose.orientation.w = orientation_val[3]

        return goal_pose

    def calc_diff(self, goal_position, current_position):
        self.get_logger().info(f"current_position value: ({self.current_position.x}, {self.current_position.y})")
        self.get_logger().info(f"goal position: ({goal_position.x}, {goal_position.y})")

        diff_x = goal_position.x - current_position.x
        diff_y = goal_position.y - current_position.y
        diff = math.sqrt(diff_x ** 2 + diff_y ** 2)
        self.get_logger().info(f"diff : {diff}")
        return diff
    
    def check_succeed(self, current_position):
        diff_dist = self.calc_diff(self.next_position, current_position)
        if diff_dist <= 0.5:
            self.get_logger().info("Moving succeeded!")
            self.verify_checkpoint()
            self.attempt_count = 0
            return True
        else:
            return self.resend_goal()
        
    def resend_goal(self):
        if self.attempt_count < MAX_ATTEMPTS:
            self.attempt_count += 1
            self.send_goal(self.next_position)
            self.get_logger().warn(f"Moving failed! Attempt {self.attempt_count}/{MAX_ATTEMPTS}")
            return False
        else:
            self.get_logger().error("Movement failed after maximum attempts.")
            self.attempt_count = 0
            self.verify_checkpoint()
            return True
    
    def verify_checkpoint(self):
        pass

    def request_robot_arrival(self, current_point):
        robot_arrival_request = NodeNum.Request()
        self.get_logger().info(f"Arrived at {current_point}")
        robot_arrival_request.current_x = current_point[0]
        robot_arrival_request.current_y = current_point[1]
        future = self.robot_arrival_client.call_async(robot_arrival_request)
        future.add_done_callback(self.response_robot_arrival)

    def response_robot_arrival(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"response {response.success} from TM")
        except Exception as e:
            self.get_logger().error(f"arrival call failed : {e}")

    def reset_callback(self, request, response):
        self.reset()
        response.success = True
        response.message = "reset"

        return response
    
    def reset(self):
        self.get_logger().info("Reset")
        self.initialize_variables()

    def returning(self):
        self.get_logger().info("Robot returning")
        self.is_active = False

class OCRobotTask(Node):
    def __init__(self, motor_node):
        super().__init__("ocrobot_task")
        self.motor_node = motor_node
        self.short_goal_server = self.create_service(NodeNum, "shortGoal", self.short_goal_callback)

    def short_goal_callback(self, request, response):
        self.get_logger().info(f"Short goal is ({request.next_x}, {request.next_y})")
        next_point = (request.next_x, request.next_y)

        if 0 <= next_point[0] <= 4 and 0 <= next_point[1] <= 8:
            self.motor_node.next_position = self.motor_node.positions[next_point[0]][next_point[1]]
            self.get_logger().info(f"next_position : {self.motor_node.next_position.x}, {self.motor_node.next_position.y}, status : {self.motor_node.status.value}, active :{self.motor_node.is_active}")
        else:
            self.get_logger().warn(f"Invalid point!!")
            response.success = False
            return response

        self.get_logger().info(f"next point : {next_point}")

        valid_status_points = {
            RobotStatus.HOME: self.motor_node.waypoint_points,
            RobotStatus.AT_HOME: self.motor_node.waypoint_points,
            RobotStatus.DRIVING: self.motor_node.waypoint_points,
            RobotStatus.RETURNING: self.motor_node.waypoint_points + self.motor_node.home_point
        }

        if next_point in valid_status_points.get(self.motor_node.status, []):
            if self.motor_node.status in {RobotStatus.HOME, RobotStatus.AT_HOME}:
                self.motor_node.update_status("DRIVING")
            self.motor_node.update_positions(self.motor_node.next_point, next_point)
            self.motor_node.is_moving = True
            response.success = True
        else:
            self.get_logger().warn(f"Invalid positions get : {self.motor_node.next_position}")
            response.success = False

        return response
    
class OCRobotStatus(Node):
    def __init__(self, motor_node):
        super().__init__("ocrobot_status")
        self.motor_node = motor_node
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.status_pub = self.create_publisher(Int16, "/status", self.qos_profile)
        # 여기서 state 변화 받아와야 할 듯

    def status_publish(self, status):
        msg = Int16()
        msg.data = status.value
        self.status_pub.publish(msg)
        # self.get_logger().info(f"Published status: {msg.data}")

    def timer_callback(self):
        self.status_publish(self.motor_node.status)

class AmclSub(Node):
    def __init__(self, motor_node):
        super().__init__("amcl_sub_node")
        self.motor_node = motor_node
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.listener_callback, 10)

    def listener_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = euler_from_quaternion(quaternion)
        roll, pitch, yaw = euler

        self.motor_node.current_yaw = yaw
        x = position.x
        y = position.y
        self.motor_node.current_position = Position(x, y)

        # self.motor_node.get_logger().info(f"Updated from amcl_pose: current_position value: {self.motor_node.current_position}")

def main(args=None):
    rp.init(args=args)
    executor = MultiThreadedExecutor()

    robot_motor = MotorController()
    robot_status = OCRobotStatus(motor_node=robot_motor)
    amcl_sub = AmclSub(motor_node=robot_motor)
    robot_task = OCRobotTask(motor_node=robot_motor)

    executor.add_node(robot_motor)
    executor.add_node(robot_status)
    executor.add_node(robot_task)
    executor.add_node(amcl_sub)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rp.ok():
            executor.shutdown()
            robot_motor.destroy_node()
            robot_status.destroy_node()
            amcl_sub.destroy_node()
            robot_task.destroy_node()
            rp.shutdown()


if __name__ == "__main__":
    main()