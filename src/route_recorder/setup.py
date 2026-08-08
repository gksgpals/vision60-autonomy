from setuptools import find_packages, setup

package_name = 'route_recorder'

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
    description='Records traversed routes and communication recovery waypoints.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'route_recorder = route_recorder.node:main',
        ],
    },
)
