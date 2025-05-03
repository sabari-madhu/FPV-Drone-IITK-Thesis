import os
from glob import glob
from setuptools import setup

package_name = 'ardupilot_dds_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.parm")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='maintainer@ardupilot.org',
    description='Tests for the ArduPilot AP_DDS library',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "time_listener = ardupilot_dds_tests.time_listener:main",
            "plane_waypoint_follower = ardupilot_dds_tests.plane_waypoint_follower:main",
	    "square = ardupilot_dds_tests.square:main",
	    "mavlink_takeoff_node = ardupilot_dds_tests.mavlink_takeoff_node:main",
	    "thesis_sq = ardupilot_dds_tests.thesis_sq_test:main",
	    "cam_tr = ardupilot_dds_tests.cam_tracking:main",
	    "person_tr = ardupilot_dds_tests.person_tracking:main",
	    "th_control = ardupilot_dds_tests.th_control_node:main",
	    "th_explore = ardupilot_dds_tests.th_explore_node:main",
	    "th_track = ardupilot_dds_tests.th_tracking_node:main",
	    "th_detect = ardupilot_dds_tests.th_detection_node:main",
	    "ardu_master = ardupilot_dds_tests.ardu_master_node:main",
	    "detection = ardupilot_dds_tests.detection_node:main",
        ],
    },
)
