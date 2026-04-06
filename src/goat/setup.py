import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'goat'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/sim_goat_config.yaml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 Isaac Sim GOAT control package',
    license='TODO',
    entry_points={
        'console_scripts': [
            'topic_io_node = goat.nodes.topic_io_node:main',
            'control_node = goat.nodes.control_node:main',
            'nsc_control_node = goat.nodes.nsc_control_node:main',
            'nsc_control_node_2 = goat.nodes.nsc_control_node_2:main',
            'nsc_plotter = goat.nodes.nsc_plotter:main',
        ],
    },
)
