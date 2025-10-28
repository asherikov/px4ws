from setuptools import setup

package_name = 'px4_offboard_demo_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/cdinit_services', ['cdinit_services/px4_offboard_demo_py_node']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS2 package for PX4 takeoff and velocity control',
    license='MIT',
    extras_require={
        'test': ['pytest']
    },
    entry_points={
        'console_scripts': [
            'demo_node = px4_offboard_demo_py.demo_node:main',
        ],
    },
)
