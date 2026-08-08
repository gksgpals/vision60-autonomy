from setuptools import find_packages, setup

package_name = 'comm_recovery_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kookmin AI Lab',
    maintainer_email='support@example.com',
    description='Communication-loss detection and recovery state machine.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'comm_recovery_manager = comm_recovery_manager.node:main',
            'recovery_path_follower = comm_recovery_manager.path_follower:main',
            'reentry_path_follower = '
            'comm_recovery_manager.reentry_follower:main',
            'communication_channel_manager = '
            'comm_recovery_manager.channel_manager:main',
            'safety_velocity_gate = '
            'comm_recovery_manager.safety_velocity_gate:main',
            'motion_lock_adapter = '
            'comm_recovery_manager.motion_lock_adapter:main',
            'exploration_safety_gate = '
            'comm_recovery_manager.exploration_gate:main',
        ],
    },
)
