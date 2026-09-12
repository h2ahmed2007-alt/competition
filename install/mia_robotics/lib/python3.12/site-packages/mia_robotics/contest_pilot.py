#libraries required for the program
import rclpy
from rclpy.node import Node

#to control the motion
from geometry_msgs.msg import Twist   


#library i already downloaded to get direct input from key presses withot pressing enter
from pynput import keyboard


class controller_node(Node):

    def __init__(self):
        super().__init__('controller_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)   
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)               #to listen to key presses
        self.listener.start()                                                   # start listening!!


    def on_press(self, key):                                # the function we used earlier in the listener , key is the keyboard input

        if key == keyboard.Key.up or getattr(key, 'char', None) == 'w' :

            twist = Twist()                                 #making a Twist type ros msg
            twist.linear.x = 1.0                            
            twist.linear.y = 0.0
            self.publisher.publish(twist)  


        elif key == keyboard.Key.down or getattr(key, 'char', None) == 's' : 

            twist = Twist()
            twist.linear.x = -1.0
            twist.linear.y = 0.0
            self.publisher.publish(twist)  

        elif getattr(key, 'char', None) == 'd' : 

            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = -1.0
            self.publisher.publish(twist)  

        
        elif getattr(key, 'char', None) == 'a' : 

            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 1.0
            self.publisher.publish(twist)  

        elif key == keyboard.Key.left:

            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 1.0
            self.publisher.publish(twist)
            
        elif key == keyboard.Key.right:

            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = -1.0
            self.publisher.publish(twist)
            
    def on_release(self, key) :
        arrows = [keyboard.Key.left,keyboard.Key.right,keyboard.Key.up,keyboard.Key.down]
        
        move_keys = ['w', 'd', 's', 'a']

        char = getattr(key, 'char', None)

        if key in arrows or char in move_keys :

            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)



def main(args=None):
    rclpy.init(args=args)

    controller = controller_node()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
