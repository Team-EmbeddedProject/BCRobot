import sys
sys.path.append('/home/pi/MasterPi/')
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import HiwonderSDK.Sonar as Sonar

class SensorProcessing(Node):
    def __init__(self):
        super().__init__('sensor_processing')
        self.camera = None
        self.resolution = (640, 480)
        self.cam_opened = False

        # camera
        self.camera_open()
        self.bridge = CvBridge()

        # sonar
        self.sonar = Sonar.Sonar()
        
        self.camera_publisher_ = self.create_publisher(Image, 'camera_image', 10)
        self.sonar_publisher_ = self.create_publisher(Float32, 'sonar_distance', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
    def camera_open(self):
        try:
            self.camera = cv2.VideoCapture('http://127.0.0.1:8080/?action=stream')
            # self.cap.set(cv2.CAP_PROP_FPS, 30)
            # self.cap.set(cv2.CAP_PROP_SATURATION, 40)
            self.cam_opened = True
            self.get_logger().info('Turn on camera')
        except Exception as e:
            print('Fail to turn on camera:', e)

    def camera_close(self):
        try:
            self.cam_opened = False
            time.sleep(0.2)
            if self.camera is not None:
                self.camera.release()
                time.sleep(0.05)
            self.camera = None
        except Exception as e:
            print('Fail to turn off camera:', e)

    def timer_callback(self):
        # camera image
        if self.cam_opened:
            ret, frame = self.camera.read()
            if ret:
                image_message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.camera_publisher_.publish(image_message)
                # self.get_logger().info('Publishing camera image')
            else:
                self.get_logger().error('Failed to capture image from camera')
        
        # sonar data
        dist = self.sonar.getDistance() / 10.0
        distance_msg = Float32()
        distance_msg.data = dist
        self.sonar_publisher_.publish(distance_msg)
        self.get_logger().info(f'Publishing sonar distance: {dist:.2f} cm')

    def destroy_node(self):
        self.camera_close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    sensor_procesing = SensorProcessing()
    try:
        rclpy.spin(sensor_procesing)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_procesing.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        