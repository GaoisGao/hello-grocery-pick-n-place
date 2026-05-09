from setuptools import find_packages, setup

package_name = 'ur3e_demo'

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
            'move_ur3e = ur3e_demo.ur3e_move_client:main',
            'move_client_with_env = ur3e_demo.move_client_with_env:main',
	    'hardcoded_pick_bt = ur3e_demo.hardcoded_pick_bt:main',
        ],
    },
)
