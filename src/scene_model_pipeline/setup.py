# Copyright 2026 Kookmin AI Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from setuptools import find_packages, setup

package_name = 'scene_model_pipeline'

setup(
    name=package_name,
    version='0.7.0',
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
    description='Traceable offline colored-cloud, voxel, and mesh pipeline.',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'build_scene_model = scene_model_pipeline.cli:main',
            'generate_synthetic_scene = '
            'scene_model_pipeline.synthetic:main',
            'generate_synthetic_mcap = '
            'scene_model_pipeline.synthetic_bag:main',
            'build_cumulative_scene = '
            'scene_model_pipeline.sequence_cli:main',
            'build_command_view = '
            'scene_model_pipeline.overlay_cli:main',
            'generate_synthetic_overlay = '
            'scene_model_pipeline.synthetic_overlay:main',
            'convert_oxford_spires_sample = '
            'scene_model_pipeline.oxford_spires_cli:main',
            'build_integrated_replay = '
            'scene_model_pipeline.integrated_replay:main',
            'validate_integrated_replay = '
            'scene_model_pipeline.integrated_replay:validation_main',
            'build_mission_overlay = '
            'scene_model_pipeline.mission_overlay:main',
        ],
    },
)
