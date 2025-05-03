import os, glob
from setuptools import setup

package_name = 'sample_code_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob.glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sabari',
    maintainer_email='sabari@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone1 = sample_code_package.offboard_control_sample1:main',
            'drone2 = sample_code_package.offboard_control_sample2:main',
            'multi_drone_custom1 = sample_code_package.offboard_control_multi_drones_sample:main'
            'multi_drone_custom2 = sample_code_package.offboard_control_multi_drones_read_sensors_sample:main',
            'multi_drone_custom3 = sample_code_package.offboard_control_multi_drones_rplidar_sample:main',
            'drone_twist1 = sample_code_package.offboard_control_twist_sample:main',
            'drone_twist2 = sample_code_package.offboard_control_twist_msg_based_sample:main',
            'processes = sample_code_package.processes:main'
        ],
    },
)
