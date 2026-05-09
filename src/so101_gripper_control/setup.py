from setuptools import find_packages, setup

package_name = 'so101_gripper_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools','feetech-servo-sdk',],
    zip_safe=True,
    maintainer='roboticslab',
    maintainer_email='roboticslab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
	    'gripper_node = so101_gripper_control.gripper_node:main',
        ],
    },
)
