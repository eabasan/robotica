#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import yaml
import os


class PathPublisher(Node):
    
    def __init__(self):
        super().__init__('path_publisher')
        
        # Declare parameters
        self.declare_parameter('path_file', 'path')
        self.declare_parameter('path_frame', 'odom')
        
        # Get parameters
        path_file = self.get_parameter('path_file').value
        path_frame = self.get_parameter('path_frame').value
        
        # Create publisher for the path
        self.path_publisher = self.create_publisher(Path, 'path', 10)
        
        # Create a timer to publish the path at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_path)
        
        # Initialize path message
        self.path = Path()
        self.path.header.frame_id = path_frame
        self.seq = 0
        
        # Load the path from the yaml file
        self.load_path(path_file)
        
    
    def load_path(self, path_file):
        """
        Load path from yaml file
        path_file: name of the yaml file (with .yaml extension)
        """
        if not path_file.endswith('.yaml'):
            self.get_logger().warn(f"Path file should have .yaml extension: {path_file}")
            path_file += '.yaml'

        try:
            with open(path_file, 'r') as f:
                self.get_logger().info(f"Loading path from file: {path_file}")
                data = yaml.safe_load(f)
                
            # Get the path goals
            if 'path' in data:
                goals = sorted(data['path'].items())
                seq_goals = 0
                
                for goal_name, goal_data in goals:
                    pose = PoseStamped()
                    pose.header.frame_id = self.path.header.frame_id
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = float(goal_data['x'])
                    pose.pose.position.y = float(goal_data['y'])
                    pose.pose.position.z = 0.0
                    
                    self.path.poses.append(pose)
                    self.get_logger().info(
                        f"Added point {pose.pose.position.x}, {pose.pose.position.y}"
                    )
                    seq_goals += 1
            else:
                self.get_logger().warn("No 'path' key found in yaml file")
                
        except FileNotFoundError:
            self.get_logger().error(f"File not found: {path_file}")
        except Exception as e:
            self.get_logger().error(f"Error loading path file: {e}")
    
    
    def publish_path(self):
        """Publish the path at regular intervals"""
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path)
        self.seq += 1


def main(args=None):
    rclpy.init(args=args)
    
    path_publisher = PathPublisher()
    
    path_publisher.get_logger().info("Path publisher started")
    
    try:
        rclpy.spin(path_publisher)
    except KeyboardInterrupt:
        path_publisher.get_logger().info("Path publisher stopped by user")
    finally:
        path_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
