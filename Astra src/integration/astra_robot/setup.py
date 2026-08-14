import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'astra_robot'


def data_files_for(subdir):
    """Install every file in a subdir into share/astra_robot/<subdir>."""
    return (os.path.join('share', package_name, subdir),
            [f for f in glob(os.path.join(subdir, '*')) if os.path.isfile(f)])


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        data_files_for('launch'),
        data_files_for('urdf'),
        data_files_for('config'),
        data_files_for('worlds'),
        data_files_for('maps'),
        data_files_for('meshes'),
        data_files_for('rviz'),
        data_files_for('tools'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sanya',
    maintainer_email='sanya.bansal.115@gmail.com',
    description='ASTRA - autonomous spacecraft navigation with ROS 2, Gazebo and Nav2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'astra_navigator = astra_robot.astra_navigator:main',
            'demo_tour = astra_robot.demo_tour:main',
        ],
    },
)
