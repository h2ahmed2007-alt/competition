import rclpy
import cv2
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        self.frame = None

        self.target_pub = self.create_publisher(String, '/target_type', 10)

        self.camera_sub = self.create_subscription(
            Image,
            '/mono/image',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

    def image_callback(self, msg):


        self.frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='mono8'
            )

            
    def read(self):

        return self.frame
    


