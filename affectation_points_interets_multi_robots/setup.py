from setuptools import setup

package_name = 'affectation_points_interets_multi_robots'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Prend en entrée les coordonnées des robot et la carte slam, puis sort les coordonnées où les robots doivent aller',
    license='MIT',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/affectation_points_interets.py']),
    ],
    entry_points={
        'console_scripts': [
            'affectation_points_interets = affectation_points_interets_multi_robots.affectation_points_interets:main',
        ],
    },
)

