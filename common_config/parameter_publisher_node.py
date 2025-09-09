import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray


class ParameterPublisherNode(Node):
    def __init__(self):
        super().__init__('parameter_publisher_node')

        # Define constants
        self.OBSTACLE_CENTER = [0.4, 0.25, 0.5]
        #self.OBSTACLE_CENTER = [0.0, 0.0, 0.0]
        self.OBSTACLE_RADIUS = 0.05
        self.TARGET_POSITION = [0.4, 0.4, 0.5]
        self.WORKSPACE_RADIUS = 0.85

        # Publishers
        self.obstacle_center_pub = self.create_publisher(Float32MultiArray, '/obstacle_center', 10)
        self.obstacle_radius_pub = self.create_publisher(Float32, '/obstacle_radius', 10)
        self.target_position_pub = self.create_publisher(Float32MultiArray, '/target_position', 10)
        self.workspace_radius_pub = self.create_publisher(Float32, '/workspace_radius', 10)

        # Start periodic publishing timer (every 1 second)
        self.publish_timer = self.create_timer(1.0, self.publish_parameters)

        # Start a one-shot timer to stop publishing after 5 seconds
        self.stop_timer = self.create_timer(5.0, self.stop_publishing)

        self.get_logger().info("Parameter Publisher Node started. Publishing every second for 5 seconds.")

    def publish_parameters(self):
        # Publish obstacle center
        obstacle_center_msg = Float32MultiArray()
        obstacle_center_msg.data = self.OBSTACLE_CENTER
        self.obstacle_center_pub.publish(obstacle_center_msg)

        # Publish obstacle radius
        obstacle_radius_msg = Float32()
        obstacle_radius_msg.data = self.OBSTACLE_RADIUS
        self.obstacle_radius_pub.publish(obstacle_radius_msg)

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
        self.get_logger().info("Stopped publishing parameters after 5 seconds.")
        rclpy.shutdown()  # Automatically shut down the node


def main(args=None):
    rclpy.init(args=args)
    node = ParameterPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
