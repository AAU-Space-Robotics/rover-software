#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math
import numpy as np

class JoyToVelNode(Node):
    def __init__(self):
        super().__init__('joy_to_vel_converter')

        # Declare parameters for input and output topics
        self.declare_parameter('joy_topic', '/local/joy')
        self.declare_parameter('twist_topic', '/joystick/cmd_vel')

        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        twist_topic = self.get_parameter('twist_topic').get_parameter_value().string_value
        
        # Subscriber
        self.subscription = self.create_subscription(
            Joy,
            joy_topic,
            self.listener_callback,
            10)
        
        # Publisher
        self.publisher_ = self.create_publisher(
            Twist,
            twist_topic,
            10)
        
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.speed_multi = 1

        self.get_logger().info(f"Joy to vel converter node started. Subscribing to '{joy_topic}' and publishing to '{twist_topic}'.")

    def listener_callback(self, msg):

        ### Convert the joystick input to linear and angular velocities ###
        
        linear_vel = msg.axes[1] # Left stick up/down
        angular_vel = msg.axes[0] # Left stick left/right
        right_stick_x = msg.axes[3] # right stick left/right


        self.speed_multi = self.check_ABXY_pressed(msg)
   

        # Strech the circle to a square
        linear_vel, angular_vel = self.circle_to_square(linear_vel, angular_vel)

        # Turn on point
        if abs(right_stick_x)>0.0001:
            # Create and publish the message
            twist = Twist()
            twist.angular.z = right_stick_x*self.speed_multi*4
            twist.angular.x = right_stick_x*self.speed_multi*4
            

            self.publisher_.publish(twist)
        else:
            # Create and publish the message
            twist = Twist()
            twist.linear.x = linear_vel*self.speed_multi
            twist.angular.z = angular_vel*self.speed_multi

            self.publisher_.publish(twist)



        # If Y button is pressed, stop the robot
        # TODO - Implement a service that stop the robot over can bus
        
        # if msg.buttons[3] == 1:
        #     twist.linear.x = 0
        #     twist.angular.z = 0
        #     self.publisher_.publish(twist)


    def check_ABXY_pressed(self, msg):
        if msg.buttons[0]==1:
            return 1.0
        if msg.buttons[1]==1:
            return 0.3
        if msg.buttons[2]==1:
            return 0.8
        if msg.buttons[3]==1:
            return 0.5
        return self.speed_multi

    def circle_to_square(self, x, y):
        # Ensure the point (x, y) lies within the unit circle
        if x**2 + y**2 > 1:
            x = x / math.sqrt(x**2 + y**2)
            y = y / math.sqrt(x**2 + y**2)

        new_x = x
        new_y = y

        if x**2 > y**2:
            # The point lies in the right half of the circle
            new_x = np.sign(x)* math.sqrt(x**2 + y**2)
        elif y != 0:
            new_x = np.sign(y)* (x / y) * math.sqrt(x**2 + y**2)

        if y**2 > x**2:
            # The point lies in the upper half of the circle
            new_y = np.sign(y)* math.sqrt(x**2 + y**2)
        elif x != 0:
            new_y = np.sign(x)* (y / x) * math.sqrt(x**2 + y**2)

        return new_x*4, new_y*4

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    # Create the node
    joy_to_vel_converter = JoyToVelNode()
    # Spin the node so the callback function is called.
    rclpy.spin(joy_to_vel_converter)
    # Destroy the node and shutdown the ROS client
    joy_to_vel_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
