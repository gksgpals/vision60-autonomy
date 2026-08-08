from setuptools import find_packages, setup

package_name = 'vision60_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kookmin AI Lab',
    maintainer_email='support@example.com',
    description='Safety-gated ROS 2 adapter for the Vision 60 SDK.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'vision60_bridge = vision60_bridge.node:main',
            'bridge_integration_probe = '
            'vision60_bridge.integration_probe:main',
        ],
    },
)
