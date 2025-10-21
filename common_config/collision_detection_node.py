#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
from geometry_msgs.msg import PoseStamped
import numpy as np
import math


class CollisionDetectionNode(Node):
    def __init__(self):
        super().__init__('collision_detection_node')
        
        # Initialize collision counter
        self.collision_count = 0
        self.currently_in_collision = False
        
        # Storage for obstacles
        self.spherical_obstacles = []
        self.cylindrical_obstacles = []
        self.cylinder_base = None
        
        # Current end effector position
        self.end_effector_position = None
        
        # Publisher for collision count
        self.collision_count_pub = self.create_publisher(
            Int32,
            '/collision_detection/collision_count',
            10
        )
        
        # Subscribers
        self.spherical_obstacles_sub = self.create_subscription(
            Float32MultiArray,
            '/spherical_obstacles',
            self.spherical_obstacles_callback,
            10
        )
        
        self.cylindrical_obstacles_sub = self.create_subscription(
            Float32MultiArray,
            '/cylindrical_obstacles',
            self.cylindrical_obstacles_callback,
            10
        )
        
        self.cylinder_base_sub = self.create_subscription(
            Float32MultiArray,
            '/cylinder_base',
            self.cylinder_base_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'admittance_controller/pose_debug',
            self.pose_callback,
            10
        )
        
        # Timer to periodically check for collisions
        self.timer = self.create_timer(0.01, self.check_collisions)  # 100Hz
        
        self.get_logger().info('Collision Detection Node started')

    def spherical_obstacles_callback(self, msg):
        """Parse spherical obstacles from Float32MultiArray"""
        self.spherical_obstacles = []
        data = msg.data
        
        # Each spherical obstacle has 4 values: x, y, z, radius
        for i in range(0, len(data), 4):
            if i + 3 < len(data):
                center = [data[i], data[i+1], data[i+2]]
                radius = data[i+3]
                if radius > 0:  # Only consider obstacles with positive radius
                    self.spherical_obstacles.append({
                        'center': center,
                        'radius': radius
                    })
    
    def cylindrical_obstacles_callback(self, msg):
        """Parse cylindrical obstacles from Float32MultiArray"""
        self.cylindrical_obstacles = []
        data = msg.data
        
        # Each cylindrical obstacle has 5 values: x, y, z, radius, height
        for i in range(0, len(data), 5):
            if i + 4 < len(data):
                center = [data[i], data[i+1], data[i+2]]
                radius = data[i+3]
                height = data[i+4]
                if radius > 0 and height > 0:  # Only consider obstacles with positive dimensions
                    self.cylindrical_obstacles.append({
                        'center': center,
                        'radius': radius,
                        'height': height
                    })
    
    def cylinder_base_callback(self, msg):
        """Parse cylinder base from Float32MultiArray"""
        data = msg.data
        if len(data) >= 5:
            center = [data[0], data[1], data[2]]
            radius = data[3]
            height = data[4]
            if radius > 0 and height > 0:
                self.cylinder_base = {
                    'center': center,
                    'radius': radius,
                    'height': height
                }
    
    def pose_callback(self, msg):
        """Update end effector position from pose message"""
        self.end_effector_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
    
    def check_sphere_collision(self, ee_pos, obstacle):
        """Check if end effector collides with a spherical obstacle"""
        center = obstacle['center']
        radius = obstacle['radius']
        
        distance = math.sqrt(
            (ee_pos[0] - center[0])**2 +
            (ee_pos[1] - center[1])**2 +
            (ee_pos[2] - center[2])**2
        )
        
        return distance <= radius
    
    def check_cylinder_collision(self, ee_pos, obstacle):
        """Check if end effector collides with a cylindrical obstacle"""
        center = obstacle['center']
        radius = obstacle['radius']
        height = obstacle['height']
        
        # Check if within height range
        if ee_pos[2] < center[2] or ee_pos[2] > center[2] + height:
            return False
        
        # Check radial distance from cylinder axis (assuming cylinder axis is along z)
        radial_distance = math.sqrt(
            (ee_pos[0] - center[0])**2 +
            (ee_pos[1] - center[1])**2
        )
        
        return radial_distance <= radius
    
    def publish_collision_count(self):
        """Publish the current collision count"""
        msg = Int32()
        msg.data = self.collision_count
        self.collision_count_pub.publish(msg)
    
    def check_collisions(self):
        """Main collision checking function"""
        if self.end_effector_position is None:
            return
        
        collision_detected = False
        
        # Check spherical obstacles
        for obstacle in self.spherical_obstacles:
            if self.check_sphere_collision(self.end_effector_position, obstacle):
                collision_detected = True
                break
        
        # Check cylindrical obstacles
        if not collision_detected:
            for obstacle in self.cylindrical_obstacles:
                if self.check_cylinder_collision(self.end_effector_position, obstacle):
                    collision_detected = True
                    break
        
        # # Check cylinder base -> Not needed anymore
        # if not collision_detected and self.cylinder_base is not None:
        #     if self.check_cylinder_collision(self.end_effector_position, self.cylinder_base):
        #         collision_detected = True
        
        # Update collision state and count
        if collision_detected and not self.currently_in_collision:
            # New collision detected
            self.collision_count += 1
            self.currently_in_collision = True
            self.get_logger().info(f'Collision detected! Total collisions: {self.collision_count}')
            # Publish updated collision count
            self.publish_collision_count()
        elif not collision_detected and self.currently_in_collision:
            # End effector is out of collision
            self.currently_in_collision = False
            self.get_logger().info('End effector is now clear of obstacles')
        
        # Always publish collision count for continuous recording
        self.publish_collision_count()


def main(args=None):
    rclpy.init(args=args)
    node = CollisionDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Final collision count: {node.collision_count}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()