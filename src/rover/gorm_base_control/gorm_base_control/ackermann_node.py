#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math
import numpy as np

class AckermannNode(Node):
    def __init__(self):
        super().__init__('ackermann_node')
        
        # Subscriber
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        
        # Publisher
        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/motor_commands',
            10)
        
        # Timer and timestamp
        self.last_message_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.check_timeout)
        
        self.get_logger().info("Ackermann node started successfully")

    def listener_callback(self, msg):

        self.last_message_time = self.get_clock().now()
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        if abs(msg.angular.x) > 0:
            steering_angles, wheel_velocities = self.turn_on_spot(angular_vel)
        else:
            # Your Ackermann function here, assume it returns steering_angles and wheel_velocities
            steering_angles, wheel_velocities = self.ackermann_steering(linear_vel, angular_vel)
            #self.get_logger().info("Steering angles: " + str(steering_angles))

        # Convert from numpy arrays to lists
        steering_angles = [float(angle) for angle in steering_angles]
        wheel_velocities = [float(vel) for vel in wheel_velocities]
        # Create and publish the message
        motor_commands = Float64MultiArray()
        motor_commands.data = wheel_velocities + steering_angles
        self.publisher_.publish(motor_commands)

            


    def turn_on_spot(self, ang_vel, wheel_radius=0.12, d_lr=0.894, d_fr=0.77, L=0.849, offset=-0.0135):
        """
        Turn the vehicle on the spot (pure rotation).
        Returns steering angles and wheel velocities.
        """
        # Steering angles (wheels at max deflection)
        theta_FL = -np.pi/4
        theta_FR = np.pi/4
        theta_RL = np.pi/4
        theta_RR = -np.pi/4
        steering_angles = np.array([theta_FL, theta_FR, theta_RL, theta_RR]) * -1

        # Wheel velocities for pure spin
        V_FL = -(ang_vel)
        V_FR = -(ang_vel)
        V_ML = -(ang_vel)
        V_MR = -(ang_vel)
        V_RL = -(ang_vel)
        V_RR = -(ang_vel)

        wheel_velocities = np.array([V_FL, V_FR, V_ML, V_MR, V_RL, V_RR]) / (wheel_radius * 2)
        return steering_angles, wheel_velocities


    def ackermann_steering(self, lin_vel, ang_vel, L=0.849, d_lr=0.894, d_fr=0.77):
        offset = -0.0135
        wheel_radius = 0.12
        minimum_radius = 0.8
        # Checking the direction of the linear and angular velocities
        direction = -1 if lin_vel < 0 else 1
        turn_direction = -1 if ang_vel < 0 else 1
    
        # Taking the absolute values
        lin_vel = abs(lin_vel)
        ang_vel = abs(ang_vel)
    
        # If the angular velocity is 0, then the steering angles are 0
        if ((ang_vel != 0.0)):
            turning_radius = lin_vel / ang_vel
        else:
            turning_radius = np.inf
        # turning_radius = np.where(not_zero_condition, lin_vel / ang_vel, np.inf)
    
        if turning_radius < minimum_radius:
            turning_radius = minimum_radius
    
        # Different turning radii for the different wheels
        R_ML = turning_radius - (d_lr / 2) * turn_direction
        R_MR = turning_radius + (d_lr / 2) * turn_direction
        R_FR = turning_radius + (d_fr / 2) * turn_direction
        R_FL = turning_radius - (d_fr / 2) * turn_direction
        R_RR = turning_radius + (d_fr / 2) * turn_direction
        R_RL = turning_radius - (d_fr / 2) * turn_direction
    
        # Steering angles
        theta_FL = np.where(turning_radius < minimum_radius,
                            -np.pi/4,
                            np.arctan2((L/2)-offset, R_FL) * turn_direction)
        # math.atan2((L/2)-offset, R_FL) * turn_direction
        theta_FR = np.where(turning_radius < minimum_radius,
                            np.pi/4,
                            np.arctan2((L/2)-offset, R_FR) * turn_direction)
        theta_ML = 0  # middle wheels don't steer
        theta_MR = 0  # middle wheels don't steer
        theta_RL = np.where(turning_radius < minimum_radius,
                            np.pi/4,
                            np.arctan2((L/2)+offset, R_RL) * -turn_direction)
        theta_RR = np.where(turning_radius < minimum_radius,
                            -np.pi/4,
                            np.arctan2((L/2)+offset, R_RR) * -turn_direction)
    
        # Array of steering angles, adjusted for direction and turning direction
        steering_angles = np.array([theta_FL, theta_FR, theta_RL, theta_RR]) * -1
    
        # Wheel velocities
        V_FL = np.where(turning_radius < minimum_radius,
                        -(ang_vel)*turn_direction,
                        np.where(ang_vel == 0, lin_vel, (R_FL * ang_vel)) * direction)
        # (ang_vel * R_FL) * direction
        V_FR = np.where(turning_radius < minimum_radius,
                        -(ang_vel)*turn_direction,
                        np.where(ang_vel == 0, -lin_vel, -(R_FR * ang_vel)) * direction)
        V_ML = np.where(turning_radius < minimum_radius,
                        -(ang_vel)*turn_direction,
                        np.where(ang_vel == 0, lin_vel, (R_ML * ang_vel)) * direction)
        V_MR = np.where(turning_radius < minimum_radius,
                        -(ang_vel)*turn_direction,
                        np.where(ang_vel == 0, -lin_vel, -(R_MR * ang_vel)) * direction)
        V_RL = np.where(turning_radius < minimum_radius,
                        -(ang_vel)*turn_direction,
                        np.where(ang_vel == 0, lin_vel, (R_RL * ang_vel)) * direction)
        V_RR = np.where(turning_radius < minimum_radius,
                        -(ang_vel)*turn_direction,
                        np.where(ang_vel == 0, -lin_vel, -(R_RR * ang_vel)) * direction)
    
        # Array of wheel velocities, adjusted for direction
        wheel_velocities = np.array([V_FL, V_FR, V_ML, V_MR, V_RL, V_RR])  # * direction
        wheel_velocities = wheel_velocities / (wheel_radius*2)
        return steering_angles, wheel_velocities
        
    def check_timeout(self):
        # Check if more than 0.5 seconds have passed since the last message
        current_time = self.get_clock().now()
        time_diff = current_time - self.last_message_time
    
        if time_diff.nanoseconds > 0.5 * 1e9:
            # Stop the vehicle by sending zero commands if no message received for 0.5 seconds
            self.get_logger().info("No command received for 0.5 seconds, stopping the vehicle.")
            motor_commands = Float64MultiArray()
            motor_commands.data = [0.0] * 10  # Assuming 4 steering angles + 6 wheel velocities
            self.publisher_.publish(motor_commands)


def main(args=None):
    rclpy.init(args=args)

    node = AckermannNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
