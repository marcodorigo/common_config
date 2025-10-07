import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import Joy


class ParameterPublisherNode(Node):
    def __init__(self):
        super().__init__('parameter_publisher_node')

        # Declare the "trial" parameter
        self.declare_parameter('trial', 'default')
        trial = self.get_parameter('trial').get_parameter_value().string_value

        # Define environment setups
        if trial == 'train':
            self.SPHERICAL_OBSTACLES = [
                {"center": [0.0, 0.0, 0.0], "radius": 0.0},
                {"center": [0.0, 0.0, 0.0], "radius": 0.0},
                {"center": [0.0, 0.0, 0.0], "radius": 0.0}
            ]
            self.CYLINDER_BASE = {"center": [0.0, 0.0, 0.0], "radius": 0.3, "height": 3.0}
            self.TARGET_POSITION = [0.1, 0.6, 0.45]
            self.WORKSPACE_RADIUS = 0.9
            self.STARTING_POSITION = [0.0, 0.0, 0.0]

        elif trial == 'easy':
            self.SPHERICAL_OBSTACLES = [
                {"center": [-0.1, 0.45, 0.3], "radius": 0.1},
                {"center": [0.0, 0.0, 0.0], "radius": 0.0},
                {"center": [0.0, 0.0, 0.0], "radius": 0.0}
            ]
            self.CYLINDER_BASE = {"center": [0.0, 0.0, 0.0], "radius": 0.3, "height": 3.0}
            self.TARGET_POSITION = [-0.3, 0.4, 0.3]
            self.WORKSPACE_RADIUS = 0.9
            self.STARTING_POSITION = [0.1, 0.6, 0.45]

        elif trial == 'hard':
            self.SPHERICAL_OBSTACLES = [
                {"center": [0.2, 0.65, 0.25], "radius": 0.07},
                {"center": [0.0, 0.45, 0.55], "radius": 0.09},
                {"center": [-0.2, 0.7, 0.35], "radius": 0.1}
            ]
            self.CYLINDER_BASE = {"center": [0.0, 0.0, 0.0], "radius": 0.3, "height": 3.0}
            self.TARGET_POSITION = [0.3, 0.5, 0.45]
            self.WORKSPACE_RADIUS = 0.9
            self.STARTING_POSITION = [-0.3, 0.4, 0.3]

        elif trial == 'singularity':
            self.SPHERICAL_OBSTACLES = [
                {"center": [0.0, 0.0, 0.0], "radius": 0.0},
                {"center": [0.0, 0.0, 0.0], "radius": 0.0}
            ]
            self.CYLINDER_BASE = {"center": [0.0, 0.0, 0.0], "radius": 0.3, "height": 3.0}
            self.TARGET_POSITION = [0.5, -0.5, 0.5]
            self.WORKSPACE_RADIUS = 0.9
            self.STARTING_POSITION = [0.3, 0.5, 0.45]
        
        else:  # Default case
            self.SPHERICAL_OBSTACLES = [
                {"center": [0.0, 0.0, 0.0], "radius": 0.0},
                {"center": [0.0, 0.0, 0.0], "radius": 0.0},
                {"center": [0.0, 0.0, 0.0], "radius": 0.0}
            ]
            self.CYLINDER_BASE = {"center": [0.0, 0.0, 0.0], "radius": 0.3, "height": 3.0}
            self.TARGET_POSITION = [0.1, 0.6, 0.45]
            self.WORKSPACE_RADIUS = 0.9
            self.STARTING_POSITION = [0.0, 0.0, 0.0]

        # Publishers
        self.spherical_obstacles_pub = self.create_publisher(Float32MultiArray, '/spherical_obstacles', 10)
        self.cylinder_base_pub = self.create_publisher(Float32MultiArray, '/cylinder_base', 10)
        self.target_position_pub = self.create_publisher(Float32MultiArray, '/target_position', 10)
        self.workspace_radius_pub = self.create_publisher(Float32, '/workspace_radius', 10)
        self.starting_position_pub = self.create_publisher(Float32MultiArray, '/starting_position', 10)

        # Publishers for simulating button presses
        self.joy_publisher = self.create_publisher(Joy, '/joy', 10)
        self.falcon_buttons_publisher = self.create_publisher(Joy, '/falcon0/buttons', 10)

        # Start periodic publishing timer
        self.publish_timer = self.create_timer(1.0, self.publish_parameters)

        # Stop publishing after 1.5 seconds
        self.stop_timer = self.create_timer(1.5, self.stop_publishing)

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

        # # Simulate button presses
        # joy_msg = Joy()
        # joy_msg.buttons = [0, 1, 1]  # Simulate buttons[1] and buttons[2] pressed
        # self.joy_publisher.publish(joy_msg)

        # falcon_msg = Joy()
        # falcon_msg.buttons = [0, 1, 1]  # Simulate buttons[1] and buttons[2] pressed
        # self.falcon_buttons_publisher.publish(falcon_msg)

        # self.get_logger().info(f"Published parameters for trial: {self.get_parameter('trial').get_parameter_value().string_value}")
        # self.get_logger().info("Simulated button presses on /joy and /falcon0/buttons.")

        # Publish starting position
        starting_position_msg = Float32MultiArray()
        starting_position_msg.data = self.STARTING_POSITION
        self.starting_position_pub.publish(starting_position_msg)

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
