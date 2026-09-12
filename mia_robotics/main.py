import rclpy
import cv2
from camera import Camera
from yolo_reading import get_detection

def main():
    rclpy.init()

    camera = Camera()

    while rclpy.ok():

        rclpy.spin_once(camera, timeout_sec=0.01)
        if camera.frame is not None:
           annotated_frame = get_detection(camera.frame)
           cv2.imshow('YOLO Detection', annotated_frame)


        if cv2.waitKey(1) & 0xFF == ord('q'):
           break

    camera.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
    
