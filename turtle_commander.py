#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import time

class turtleCommander(Node):
    def __init__(self):
        super().__init__("turtle_commander")
        self.subscriber = self.create_subscription(String,"shape_command",self.listener_callback,10)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.circle_timer = None
        self.star_timer = None
        self.spiral_timer = None
        self.star_step = 0
        self.star_points = 5 


    def listener_callback(self,msg:String):
        shape = msg.data.lower()
        
        if shape == "circle":
            self.get_logger().info(f"drawing {shape}")
            self.draw_circle()
            
        elif shape == "star":
            self.get_logger().info(f"drawing {shape}")
            self.draw_star()
            
        elif shape == "spiral":
            self.get_logger().info(f"drawing {shape}")
            self.draw_spiral()
        
        elif shape == "stop drawing":
            self.stop()
        else:
            self.get_logger().info("unknown shape")


    def draw_circle(self):
        cmd = Twist()
        cmd.linear.x = 2.0
        cmd.angular.z = 1.0
        self.circle_timer = self.create_timer(0.1, lambda: self.publisher.publish(cmd))


    def draw_star(self):
        self.star_step = 0
        self.star_timer = self.create_timer(2.0, self.star_callback) 


    def star_callback(self):
        if self.star_step >= self.star_points:
            self.star_timer.cancel()
            self.stop()
            return
        
        cmd = Twist()
       
        cmd.linear.x = 2.0
        self.publisher.publish(cmd)
        time.sleep(1.0)  

        
        cmd.linear.x = 0.0
        self.publisher.publish(cmd)
        time.sleep(0.2)

        
        cmd.angular.z = 2.5
        self.publisher.publish(cmd)
        time.sleep(1.0)

    
        cmd.angular.z = 0.0
        self.publisher.publish(cmd)

        self.star_step += 1


    def draw_spiral(self):
        self.spiral_step = 0
        self.spiral_timer = self.create_timer(0.1, self.spiral_callback)

    def spiral_callback(self):
     cmd = Twist()
     cmd.linear.x = 2.0
     cmd.angular.z = 0.5 + 0.05 * (self.spiral_step / 5.0)  
     self.publisher.publish(cmd)
     

    
     self.spiral_step += 1
         




    def stop(self):
        stop_cmd = Twist()
        self.publisher.publish(stop_cmd)

        if self.circle_timer:
            self.circle_timer.cancel()
        if self.star_timer:
            self.star_timer.cancel()
        if self.spiral_timer:
            self.spiral_timer.cancel()
        


def main(args=None):
    rclpy.init(args=args)
    node = turtleCommander()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
