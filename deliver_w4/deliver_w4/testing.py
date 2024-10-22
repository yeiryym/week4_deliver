#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class TurtleBotVacuumCleaner(Node):

    def __init__(self):
        super().__init__('turtlebot_vacuum_cleaner')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscriber_ = self.create_subscription(Odometry, '/odom', self.update_pose, 10)
        self.timer = self.create_timer(0.1, self.clean_area)
        self.cmd = Twist()

        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        
        self.linear_velocity = 0.2  
        self.angular_velocity = 0.5  
        self.radius_increase_rate = 0.003  
        self.max_radius = 10.0  
       
        self.stop_cleaning = False  

    def update_pose(self, msg):
        
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def clean_area(self):
        if self.stop_cleaning:
            
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            return

        # distance from the origin
        distance_from_center = math.sqrt(self.current_x**2 + self.current_y**2)

        if distance_from_center >= self.max_radius:
            
            self.get_logger().info(f"TurtleBot reached boundary at radius {distance_from_center:.2f}. Stopping.")
            self.stop_cleaning = True
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            return

        
            self.linear_velocity += self.radius_increase_rate
        else:
            self.linear_velocity = self.max_radius

        
        self.cmd.linear.x = self.linear_velocity
        self.cmd.angular.z = self.angular_velocity  

        
        self.publisher_.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    turtlebot_vacuum_cleaner = TurtleBotVacuumCleaner()

    try:
        rclpy.spin(turtlebot_vacuum_cleaner)
    except KeyboardInterrupt:
        pass

    turtlebot_vacuum_cleaner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()