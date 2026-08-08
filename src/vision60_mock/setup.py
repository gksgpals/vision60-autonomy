from setuptools import find_packages, setup

package_name = 'vision60_mock'

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
    description='Mock Vision 60 motion and communication failure scenarios.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vision60_mock = vision60_mock.node:main',
            'nav2_mock_robot = vision60_mock.nav2_robot:main',
            'follow_path_probe = vision60_mock.follow_path_probe:main',
            'recovery_pipeline_probe = '
            'vision60_mock.recovery_pipeline_probe:main',
            'integration_probe = vision60_mock.integration_probe:main',
            'mock_lidar_heartbeat = vision60_mock.lidar_heartbeat:main',
            'full_system_probe = vision60_mock.full_system_probe:main',
            'sensor_comm_fault_probe = '
            'vision60_mock.sensor_comm_fault_probe:main',
        ],
    },
)
