from setuptools import find_packages, setup

package_name = 'mission_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kookmin AI Lab',
    maintainer_email='support@example.com',
    description='Replaceable 2D detection and LiDAR 3D localization pipeline.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'mission_perception = mission_perception.node:main',
            'validate_perception_dataset = '
            'mission_perception.dataset_cli:validation_main',
            'generate_perception_dataset_fixture = '
            'mission_perception.dataset_cli:fixture_main',
            'import_dfire_dataset = '
            'mission_perception.dataset_cli:dfire_import_main',
            'generate_dfire_dataset_fixture = '
            'mission_perception.dataset_cli:dfire_fixture_main',
        ],
    },
)
