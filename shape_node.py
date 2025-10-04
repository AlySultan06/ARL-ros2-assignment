#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import String


class shapeNode(Node):
    def __init__(self):
        super().__init__("shape_node")
        self.publisher = self.create_publisher(String,"shape_command",10)
        self.waiting_input = False
        self.create_timer(0.1,self.choose_command)
        
        

    
    def choose_command(self):
        print("Choose a shape:")
        print("1. Circle")
        print("2. Star")
        print("3. Spiral")
        print("4. Stop drwaing")
        shape = input("Enter a shape: ")
        print(""" 

""")
        msg = String()
        msg.data = shape
        self.publisher.publish(msg)

       

        

    
    
        


    

        

        

  



def main(args=None):
    rclpy.init(args= args)

    node = shapeNode()
    rclpy.spin(node)
    

    rclpy.shutdown()


if __name__ == '__main__':
    main()
