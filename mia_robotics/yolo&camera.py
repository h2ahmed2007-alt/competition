import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

model = YOLO("best.pt") 

def get_detection(frame):

    results = model(frame)
    annotated_frame = results[0].plot()

    return annotated_frame
        

class camera(Node):
    def __init__(self):
        super().__init__('camera')
        self.frame = None

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
    

####### to be included in the main file ---> right now is a test file 
def main():
    rclpy.init()

    camera = camera()

    while rclpy.ok():

        rclpy.spin_once(camera, timeout_sec=0.01)
        if camera.frame is not None:
           annotated_frame = get_detection(camera.frame)
           cv2.imshow('YOLO Detection', annotated_frame)

        if cv2.waitKey(1) == 27:
            break

    camera.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
    
