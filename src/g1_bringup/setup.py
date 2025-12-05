from setuptools import setup

package_name = 'g1_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/sim_launch.py',
            'launch/sim_nav_launch.py',
            'launch/robot_launch.py',
            'launch/inspection_launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml',
            'config/slam_params.yaml',
            'config/robot_params.yaml',
        ]),
        ('share/' + package_name + '/config/rviz', [
            'config/rviz/sim.rviz',
            'config/rviz/nav.rviz',
            'config/rviz/sensor_test.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='G1 Inspector Team',
    maintainer_email='g1-inspector@unitree-project.dev',
    description='G1 Inspector launch files and configuration',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mujoco_sim = g1_bringup.mujoco_sim:main',
        ],
    },
)
