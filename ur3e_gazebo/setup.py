from setuptools import find_packages, setup
from glob import glob

package_name = 'ur3e_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex Kautz',
    maintainer_email='alex.goodheart.kautz@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'home_pose = ur3e_gazebo.home_pose:main',
            'arm_camera_localizer = ur3e_gazebo.arm_camera_localizer:main',
            'overhead_camera_localizer = ur3e_gazebo.overhead_camera_localizer:main',
            'pick_and_place = ur3e_gazebo.pick_and_place:main',
            'joint_control_panel = ur3e_gazebo.joint_control_panel:main',
        ],
    },
)
