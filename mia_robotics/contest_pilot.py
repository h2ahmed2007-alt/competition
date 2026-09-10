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
        self.listener = keyboard.Listener(on_press=self.on_press)               #to listen to key presses
        self.listener.start()                                                   # start listening!!


#the logic used below is linear x --> + is forward, - is backward
#since the movement is non holonomic and its 2d we wont use "y"


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

        elif key == keyboard.Key.right or getattr(key, 'char', None) == 'd' : 

            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = -1.0
            self.publisher.publish(twist)  

        
        elif key == keyboard.Key.left or getattr(key, 'char', None) == 'a' : 

            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 1.0
            self.publisher.publish(twist)  




def main(args=None):
    rclpy.init(args=args)

    controller = controller_node()

    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()