from setuptools import setup

package_name = 'g1_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='G1 Inspector Team',
    maintainer_email='g1-inspector@unitree-project.dev',
    description='G1 Inspector sensor capture and image processing',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_camera = g1_perception.sim_camera:main',
            'sim_lidar = g1_perception.sim_lidar:main',
            'sim_imu = g1_perception.sim_imu:main',
            'lidar_to_scan = g1_perception.lidar_to_scan:main',
            'depth_to_pointcloud = g1_perception.depth_to_pointcloud:main',
        ],
    },
)
