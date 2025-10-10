from setuptools import setup

package_name = 'common_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=['common_config'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/parameter_publisher_launch.py']),  # Add launch file if applicable
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marco',
    maintainer_email='marco3.dorigo@mail.polimi.it',
    description='Common configuration package for ROS 2 nodes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parameter_publisher_node = common_config.parameter_publisher_node:main',
            'visualizer_node = common_config.visualizer_node:main',
            'collision_detection_node = common_config.collision_detection_node:main',
        ],
    },
)