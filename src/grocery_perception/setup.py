from setuptools import find_packages, setup

package_name = 'grocery_perception'

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
    install_requires=['setuptools'],
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
            'object_pose_estimator = grocery_perception.object_pose_estimator:main',
	        'object_pose_transformer = grocery_perception.object_pose_transformer:main',
            'pick_planner = grocery_perception.pick_planner:main',
            'pick_executor = grocery_perception.pick_executor:main',
	    'ik_checker = grocery_perception.ik_checker:main',
        ],
    },
)
