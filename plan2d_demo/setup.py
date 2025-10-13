from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'plan2d_demo'

my_maps = glob('maps/*')

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rviz_plan_only.launch.py']),
        ('share/' + package_name + '/params', ['params/nav2_params.yaml']),
        ('share/' + package_name + '/maps', my_maps),
        ('share/' + package_name + '/rviz',   ['rviz/blank.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='apka',
    maintainer_email='apka@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={             # <- 선택: pip로 테스트 의존 설치하고 싶을 때만 유지
        'test': ['pytest', 'ament-flake8', 'ament-pep257', 'ament-pytest']
    },
    entry_points={
        'console_scripts': [
            'path_client = plan2d_demo.path_client:main',
            'kinematic_sim = plan2d_demo.kinematic_sim:main',
            'robot_marker = plan2d_demo.robot_marker:main',
        ],
    },
)
