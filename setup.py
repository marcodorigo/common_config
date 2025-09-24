from setuptools import setup

package_name = 'parameter_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=['parameter_node'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'parameter_publisher_node = parameter_node.parameter_publisher_node:main',
        ],
    },
)