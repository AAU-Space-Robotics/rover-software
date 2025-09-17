#!/usr/bin/env python3

"""
GORM Rover RL Navigation Node

This node implements reinforcement learning-based navigation for the GORM rover
using a Triton Inference Server for model inference.

Project Context: This node belongs in the gorm_navigation package. Its purpose is to
provide autonomous navigation capabilities using trained RL models.

Constraints:
- Language: Python (suitable for high-level navigation logic)
- Key Inputs: Depth images from ZED camera, goal position, robot pose
- Key Outputs: Velocity commands to /autonomous/cmd_vel topic
- Uses Triton Inference Server for RL model inference
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gorm_navigation.utils.ros_to_numpy import image_to_numpy
from gorm_navigation.utils.math_utils import quat_rotate_inverse, yaw_quat
import numpy as np
from scipy.ndimage import zoom
import math
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# Triton client imports (optional - will gracefully degrade if not available)
try:
    from tritonclient.http import InferenceServerClient, InferInput, InferRequestedOutput
    TRITON_AVAILABLE = True
except ImportError:
    TRITON_AVAILABLE = False


class GormRLNavigationNode(Node):
    """
    ROS 2 node for RL-based navigation of the GORM rover.
    
    This node subscribes to depth images, robot pose, and goal positions,
    then uses a trained RL model via Triton Inference Server to generate
    velocity commands for autonomous navigation.
    """
    
    def __init__(self):
        super().__init__('gorm_rl_navigation_node')
        
        # Declare parameters
        self.declare_parameter('triton_server_url', 'localhost:8000')
        self.declare_parameter('model_name', 'IQL_depth_224_224')
        self.declare_parameter('model_version', '1')
        self.declare_parameter('inference_rate', 5.0)  # Hz
        self.declare_parameter('depth_image_height', 224)
        self.declare_parameter('depth_image_width', 224)
        self.declare_parameter('max_depth_value', 4.0)
        self.declare_parameter('enable_triton', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        
        # Get parameters
        self.triton_server_url = self.get_parameter('triton_server_url').get_parameter_value().string_value
        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.model_version = self.get_parameter('model_version').get_parameter_value().string_value
        self.inference_rate = self.get_parameter('inference_rate').get_parameter_value().double_value
        self.depth_height = self.get_parameter('depth_image_height').get_parameter_value().integer_value
        self.depth_width = self.get_parameter('depth_image_width').get_parameter_value().integer_value
        self.max_depth = self.get_parameter('max_depth_value').get_parameter_value().double_value
        self.enable_triton = self.get_parameter('enable_triton').get_parameter_value().bool_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        
        # Initialize Triton client
        self.triton_client = None
        if TRITON_AVAILABLE and self.enable_triton:
            try:
                self.triton_client = InferenceServerClient(url=self.triton_server_url, verbose=False)
                if self.triton_client.is_server_live():
                    self.get_logger().info(f"Connected to Triton server at {self.triton_server_url}")
                else:
                    self.get_logger().warn(f"Triton server at {self.triton_server_url} is not live")
                    self.triton_client = None
            except Exception as e:
                self.get_logger().error(f"Failed to connect to Triton server: {e}")
                self.triton_client = None
        else:
            self.get_logger().warn("Triton inference disabled or unavailable")
        
        # Initialize TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize data storage
        self.depth_data: np.ndarray = None
        self.goal_data: np.ndarray = np.array([9.0, 0.0, 0.0])  # Default goal
        self.pose_data: np.ndarray = None
        self.prev_action = np.array([0.0, 0.0])
        self.depth_image_shape = np.array([self.depth_height, self.depth_width])
        
        # Setup Publishers
        self.velocity_publisher = self.create_publisher(
            Twist, '/autonomous/cmd_vel', 10)
        
        # Setup Subscribers
        # Subscribe to ZED depth topic
        self.depth_subscription = self.create_subscription(
            Image, '/zed_front/zed/depth/depth_registered', 
            self.depth_callback, 10)
        
        # Subscribe to goal position (from Nav2 or manual goal setting)
        self.goal_subscription = self.create_subscription(
            PoseStamped, '/goal_pose', 
            self.goal_callback, 10)
        
        # Note: We now use TF2 to get the base_link pose instead of subscribing to pose topics
        
        # Create inference timer
        timer_period = 1.0 / self.inference_rate
        self.inference_timer = self.create_timer(timer_period, self.inference_callback)
        
        self.get_logger().info("GORM RL Navigation node started successfully")
        self.get_logger().info(f"Inference rate: {self.inference_rate} Hz")
        self.get_logger().info(f"Depth image target size: {self.depth_height}x{self.depth_width}")
        self.get_logger().info(f"Using TF2 to get pose: {self.odom_frame} -> {self.base_frame}")
    
    def depth_callback(self, msg: Image):
        """Handle incoming depth image data."""
        self.get_logger().debug("Received depth image message")
        try:
            self.depth_data = image_to_numpy(msg)
            if self.depth_data is not None:
                self.get_logger().debug(f"Received depth image: {self.depth_data.shape}")
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")
    
    def goal_callback(self, msg: PoseStamped):
        """Handle incoming goal position."""
        self.goal_data = np.array([
            msg.pose.position.x, 
            msg.pose.position.y, 
            msg.pose.position.z
        ])
        self.get_logger().info(f"New goal received: {self.goal_data}")
    
    def get_base_link_pose(self) -> np.ndarray:
        """
        Get the current pose of base_link in the odom frame using TF2.
        
        Returns:
            Pose array [x, y, z, qw, qx, qy, qz] or None if transform not available
        """
        try:
            # Look up the transform from odom to base_link
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame, 
                self.base_frame, 
                rclpy.time.Time())
            
            # Extract position and orientation
            pos = transform.transform.translation
            rot = transform.transform.rotation
            
            return np.array([
                pos.x, pos.y, pos.z,
                rot.w, rot.x, rot.y, rot.z
            ])
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Failed to get base_link transform: {e}")
            return None
    
    def prepare_observations(self) -> np.ndarray:
        """
        Prepare observation vector for RL model.
        
        Returns:
            Observation array with shape (1, 5)
        """
        # Vector from robot to goal in world frame
        target_vec = self.goal_data[:3] - self.pose_data[:3]
        
        # Vector from robot to goal in robot frame
        target_vec_r = quat_rotate_inverse(
            yaw_quat(self.pose_data[3:]), target_vec)
        
        print(np.linalg.norm(target_vec_r))
        # Prepare observation vector
        observations = np.zeros(5, dtype=np.float32)
        observations[:2] = self.prev_action  # Previous action
        observations[2] = np.linalg.norm(target_vec_r) / 9.0  # Normalized distance
        observations[3] = np.arctan2(target_vec_r[1], target_vec_r[0]) / np.pi  # Normalized angle
        observations[4] = 0.02  
        
        # Add batch dimension (1, 5)
        return observations[np.newaxis, :]
    
    def prepare_depth_input(self) -> np.ndarray:
        """
        Prepare depth image for RL model input.
        
        Returns:
            Processed depth array with shape (1, 1, height, width)
        """
        # Resize depth image to target shape
        zoom_factor = self.depth_image_shape / self.depth_data.shape
        depth = zoom(self.depth_data, zoom_factor, order=0)
        
        # Clean and clip depth data
        depth = np.nan_to_num(depth, nan=self.max_depth)
        depth = np.clip(depth, 0.0, self.max_depth)
        depth = depth / self.max_depth  # Normalize to [0, 1]
        
        # Add batch and channel dimensions (1, 1, height, width)
        return depth[np.newaxis, np.newaxis, :, :].astype(np.float32)
    
    def inference_callback(self):
        """Main inference loop - called at specified rate."""
        # Check if Triton client is available
        if self.triton_client is None:
            self.get_logger().warn("Triton client not available - publishing zero velocity")
            self.publish_zero_velocity()
            return
        
        # Check server status
        if not self.triton_client.is_server_live():
            self.get_logger().error_throttle("Triton server is not live")
            self.publish_zero_velocity()
            return
        
        # Check if all required data is available
        if self.depth_data is None:
            self.get_logger().warn("Depth data not available")
            return
        
        if self.goal_data is None:
            self.get_logger().warn("Goal data not available")
            return
        
        # Get current base_link pose using TF2
        self.pose_data = self.get_base_link_pose()
        if self.pose_data is None:
            self.get_logger().warn("Pose data not available from TF2")
            return
        
        try:
            # Prepare model inputs
            observations = self.prepare_observations()
            depth_input = self.prepare_depth_input()
            if self.depth_data is None:
                depth_input = np.zeros((1, 1, self.depth_height, self.depth_width), dtype=np.float32)
                observations = np.zeros((1, 5), dtype=np.float32)  # Fallback if depth is not available
            
            # Prepare Triton inputs
            input0 = InferInput('proprioceptive', observations.shape, 'FP32')
            input0.set_data_from_numpy(observations)
            input1 = InferInput('image', depth_input.shape, 'FP32')
            input1.set_data_from_numpy(depth_input)
            
            # Prepare outputs
            output0 = InferRequestedOutput('action', binary_data=True)

            # Run inference
            results = self.triton_client.infer(
                model_name=self.model_name,
                model_version=self.model_version,
                inputs=[input0, input1],
                outputs=[output0]
            )
            
            # Extract action from results
            action = results.as_numpy('action')[0]
            
            # Publish velocity command
            self.publish_velocity(action)
            
            # Update previous action
            self.prev_action = action
            
            # Log occasionally for debugging
            if self.get_clock().now().nanoseconds % 2000000000 < 200000000:  # Every ~2 seconds
                self.get_logger().info(f"RL Action: linear={action[0]:.3f}, angular={action[1]:.3f}")
                self.get_logger().info(f"Distance: {observations[:,2]}")
        
        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            self.publish_zero_velocity()
    
    def publish_velocity(self, action: np.ndarray):
        """Publish velocity command from RL action."""
        msg = Twist()
        msg.linear.x = float(action[0])
        msg.angular.z = float(action[1])
        self.velocity_publisher.publish(msg)
    
    def publish_zero_velocity(self):
        """Publish zero velocity (safety fallback)."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.velocity_publisher.publish(msg)


def main(args=None):
    """Main entry point for the RL navigation node."""
    rclpy.init(args=args)
    
    try:
        rl_navigation_node = GormRLNavigationNode()
        rclpy.spin(rl_navigation_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in RL navigation node: {e}")
    finally:
        if 'rl_navigation_node' in locals():
            rl_navigation_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
