from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mini_pupper_behavior'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='charlene.vernant@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_server = mini_pupper_behavior.behavior_server:main',
            'behavior_client = mini_pupper_behavior.behavior_client:main',
            'pose_controller = mini_pupper_behavior.pose_controller:main'
        ],
    },
)
