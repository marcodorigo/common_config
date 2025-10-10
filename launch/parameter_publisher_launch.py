from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the 'trial' launch argument with a default value
    trial_arg = DeclareLaunchArgument(
        'trial',
        default_value='default',
        description='Trial case to configure the environment setup'
    )

    # Node configuration
    parameter_publisher_node = Node(
        package='common_config',
        executable='parameter_publisher_node',
        name='parameter_publisher_node',
        output='screen',
        parameters=[{
            'trial': LaunchConfiguration('trial')
        }]
    )

    visualizer_node = Node(
        package='common_config',
        executable='visualizer_node',
        name='visualizer_node',
        output='screen',
    )

    collision_detection_node = Node(
        package='common_config',
        executable='collision_detection_node',
        name='collision_detection_node',
        output='screen',
    )

    return LaunchDescription([
        trial_arg,
        parameter_publisher_node,
        visualizer_node,
        collision_detection_node
    ])