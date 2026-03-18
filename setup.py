from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tb3_odom_imu_validation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jvang',
    maintainer_email='johnnyjvang@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'forward_straightness = tb3_odom_imu_validation.forward_straightness:main',
            'backward_straightness = tb3_odom_imu_validation.backward_straightness:main',
            'straightness_test = tb3_odom_imu_validation.straightness_test:main',
            'rotation_consistency_test = tb3_odom_imu_validation.rotation_consistency_test:main',
            'out_and_back_heading = tb3_odom_imu_validation.out_and_back_heading:main',
            # Added to print and reset json output
            'reset_results = tb3_odom_imu_validation.reset_results:main',
            'summary_report = tb3_odom_imu_validation.summary_report:main',
        ],
    },
)
