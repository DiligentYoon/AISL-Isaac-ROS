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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 topic I/O tester for Isaac Sim GOAT',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_topic_io = goat.test_topic_io:main',
        ],
    },
)
