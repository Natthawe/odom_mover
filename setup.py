from setuptools import find_packages, setup

package_name = 'odom_mover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cg',
    maintainer_email='cg@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_1m = odom_mover.move_1m:main',
            'move1m_turn90 = odom_mover.move1m_turn90:main',
            'rotate_to_yaw = odom_mover.rotate_to_yaw:main',
            'odom_mover = odom_mover.odom_mover:main',
            'odom_mover2 = odom_mover.odom_mover2:main',
            'odom_mover3 = odom_mover.odom_mover3:main',
            'odom_yaw_reader = odom_mover.odom_yaw_reader:main',
            'odom_yaw_plotter = odom_mover.odom_yaw_plotter:main',
            'yaw_controller = odom_mover.yaw_controller:main',
        ],
    },
)
