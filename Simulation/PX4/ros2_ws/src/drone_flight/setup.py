from setuptools import setup

package_name = 'drone_flight'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sabari',
    maintainer_email='sabarim2131@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist = drone_flight.square:main',
            'track = drone_flight.pattern_tracker_drone:main',
            'cam = drone_flight.camera_pattern_compare:main'
        ],
    },
)
