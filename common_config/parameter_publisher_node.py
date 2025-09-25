import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


class ParameterPublisherNode(Node):
    def __init__(self):
        super().__init__('parameter_publisher_node')

        # Define constants
        self.SPHERICAL_OBSTACLES = [
            {"center": [-0.3, 0.5, 0.5], "radius": 0.07},  
            {"center": [0.02, 0.42, 0.5], "radius": 0.07}  
        ]
        self.CYLINDER_BASE = {"center": [0.0, 0.0, 0.0], "radius": 0.27, "height": 4.0} 
        self.TARGET_POSITION = [0.3, 0.5, 0.45]
        self.WORKSPACE_RADIUS = 0.85

        # Publishers
        self.spherical_obstacles_pub = self.create_publisher(Float32MultiArray, '/spherical_obstacles', 10)
        self.cylinder_base_pub = self.create_publisher(Float32MultiArray, '/cylinder_base', 10)
        self.target_position_pub = self.create_publisher(Float32MultiArray, '/target_position', 10)
        self.workspace_radius_pub = self.create_publisher(Float32, '/workspace_radius', 10)

        # Start periodic publishing timer
        self.publish_timer = self.create_timer(1.0, self.publish_parameters)

        # Stop publishing after 5 seconds
        self.stop_timer = self.create_timer(5.0, self.stop_publishing)

    def publish_parameters(self):
        # Publish spherical obstacles
        spherical_obstacles_msg = Float32MultiArray()
        spherical_obstacles_data = []
        for obstacle in self.SPHERICAL_OBSTACLES:
            spherical_obstacles_data.extend(obstacle["center"])
            spherical_obstacles_data.append(obstacle["radius"])
        spherical_obstacles_msg.data = spherical_obstacles_data
        self.spherical_obstacles_pub.publish(spherical_obstacles_msg)

        # Publish cylinder base
        cylinder_base_msg = Float32MultiArray()
        cylinder_base_msg.data = self.CYLINDER_BASE["center"] + [self.CYLINDER_BASE["radius"], self.CYLINDER_BASE["height"]]
        self.cylinder_base_pub.publish(cylinder_base_msg)

        # Publish target position
        target_position_msg = Float32MultiArray()
        target_position_msg.data = self.TARGET_POSITION
        self.target_position_pub.publish(target_position_msg)

        # Publish workspace radius
        workspace_radius_msg = Float32()
        workspace_radius_msg.data = self.WORKSPACE_RADIUS
        self.workspace_radius_pub.publish(workspace_radius_msg)

        self.get_logger().info("Published parameters.")

    def stop_publishing(self):
        self.publish_timer.cancel()
        self.get_logger().info("Stopped publishing parameters.")
        rclpy.shutdown() 

def main(args=None):
    rclpy.init(args=args)
    node = ParameterPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
