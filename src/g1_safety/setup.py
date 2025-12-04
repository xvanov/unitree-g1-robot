from setuptools import setup

package_name = 'g1_safety'

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
    description='G1 Inspector E-stop and battery monitoring',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
