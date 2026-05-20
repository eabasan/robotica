#!/usr/bin/env python3
# This Python file uses the following encoding: utf-8

import math
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
import tf2_ros


class Utils():
    
    def __init__(self, tf_buffer, odom_topic=None):
        """
        Initialize the Utils class
        tf_buffer: tf2_ros.Buffer object for transform lookups
        odom_topic: optional, for backward compatibility (not used in ROS2 version)
        """
        self.tf_buffer = tf_buffer
        self.odom = Odometry()
     
    def set_odom(self, odom_msg):
        """
        Store the odometry message
        """
        self.odom = odom_msg
        

    def get_odom(self):
        """
        Return the current odometry message
        This method can be called from the robot controller
        to obtain the robot position and velocity
        in the odometry frame
        """
        return self.odom   

    
    def compute_new_xy_positions(self, xi, yi, vx, vy, theta, dt):
        """
        Compute x and y positions based on velocity
        xi: The current x position (m)
        yi: The current y position (m)
        vx: The current x velocity (m/s)
        vy: The current y velocity (m/s)
        theta: The current orientation (rad)
        dt: The timestep to take (secs)
        Returns: The new x and y positions (m)
        """
        newx = xi + (vx * math.cos(theta) + vy * math.cos(math.pi/2.0 + theta)) * dt
        newy = yi + (vx * math.sin(theta) + vy * math.sin(math.pi/2.0 + theta)) * dt
        return newx, newy
    
    
    def compute_new_theta_position(self, thetai, vth, dt):
        """
        Compute orientation based on velocity
        thetai: The current orientation (rad)
        vth: The current theta velocity (rad/s)
        dt: The timestep to take (s)
        Returns: The new orientation (rad)
        """
        return thetai + vth * dt
  

    def compute_new_velocity(self, vg, vi, a_max, dt): 
        """
        Compute velocity based on acceleration
        vg: The desired velocity, what we're accelerating up to (m/s)
        vi: The current velocity (m/s)
        a_max: An acceleration limit (m/s^2)
        dt: The timestep to take (s)
        Returns: The new velocity (m/s)
        """
        if ((vg - vi) >= 0):
            return min(vg, vi + a_max * dt)
        return max(vg, vi - a_max * dt)
  

    def transform_pose(self, pose, to_frame, logger=None):
        """
        Transform a PoseStamped to given frame
        pose: PoseStamped to be transformed
        to_frame: desired frame
        logger: optional logger for error messages
        Returns: the transformed PoseStamped
        """
        pose.header.stamp = type(pose.header.stamp)(sec=0, nanosec=0)
        pose_trans = PoseStamped()
        try:
            pose_trans = self.tf_buffer.transform(pose, to_frame)
        except tf2_ros.LookupException as e:
            if logger:
                logger.error(f"Transform lookup failed: {e}")
        except tf2_ros.ConnectivityException as e:
            if logger:
                logger.error(f"Transform connectivity failed: {e}")
        except tf2_ros.ExtrapolationException as e:
            if logger:
                logger.error(f"Transform extrapolation failed: {e}")
            
        return pose_trans
    

    def transform_point(self, point, to_frame, logger=None):
        """
        Transform a PointStamped to given frame
        point: PointStamped to be transformed
        to_frame: desired frame
        logger: optional logger for error messages
        Returns: the transformed PointStamped
        """
        point.header.stamp = type(point.header.stamp)(sec=0, nanosec=0)
        point_trans = PointStamped()
        try:
            point_trans = self.tf_buffer.transform(point, to_frame)
        except tf2_ros.LookupException as e:
            if logger:
                logger.error(f"Transform lookup failed: {e}")
        except tf2_ros.ConnectivityException as e:
            if logger:
                logger.error(f"Transform connectivity failed: {e}")
        except tf2_ros.ExtrapolationException as e:
            if logger:
                logger.error(f"Transform extrapolation failed: {e}")
            
        return point_trans
