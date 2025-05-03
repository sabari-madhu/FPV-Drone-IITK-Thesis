from glob import glob
from setuptools import find_packages, setup

package_name = 'thesis_nodes'

def read_requirements(file_path):
    try:
        print("######\n\n######")
        with open(file_path, 'r') as f:
            reqs = [line.strip() for line in f if line.strip()]
        print("Requirements read:", reqs)
        return reqs
    except Exception as e:
        print("Could not read requirements: ", e)
        return []


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',  glob('launch/*.launch.py')), # Add this line

    ],
    install_requires=['setuptools'], #read_requirements('requirements.txt'),
    zip_safe=True,
    maintainer='sabari',
    maintainer_email='sabari@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ardupilot_communicator = thesis_nodes.ArdupilotConnector:main",
            "scan = thesis_nodes.ScanPerson:main",
            "camera_module = thesis_nodes.CameraProcessing:main",
            "tracking = thesis_nodes.Tracking:main",
            "master_drone = thesis_nodes.MasterController:main",

        ],
    },
)
