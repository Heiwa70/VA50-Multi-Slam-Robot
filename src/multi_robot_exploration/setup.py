import os
from setuptools import find_packages, setup

package_name = 'multi_robot_exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', [
            os.path.join('launch', f)
            for f in os.listdir('launch')
            if os.path.isfile(os.path.join('launch', f))
        ]),
        (f'share/{package_name}/launch/nav2_bringup', [
            os.path.join('launch/nav2_bringup', f)
            for f in os.listdir('launch/nav2_bringup')
            if os.path.isfile(os.path.join('launch/nav2_bringup', f))
        ]),
        (f'share/{package_name}/params', [
            os.path.join('params', f)
            for f in os.listdir('params')
            if os.path.isfile(os.path.join('params', f))
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cyril',
    maintainer_email='cyrilgottfried11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'icp_alignment = multi_robot_exploration.icp_alignment:main',
            'multi_robot_frontier_explorer = multi_robot_exploration.multi_robot_frontier_explorer:main',
            'dynamic_laser_filter = multi_robot_exploration.dynamic_laser_filter:main',
            'teleop_arrows = multi_robot_exploration.teleop_arrows:main',
        ],
    },
)
