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

from pathlib import Path

from geometry_msgs.msg import Point
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from scene_model_pipeline.integrated_replay import build_integrated_replay
from scene_model_pipeline.synthetic_bag import ros_time, topic_metadata


def _write_mission_bag(path: Path, timestamp_ns: int) -> None:
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    writer.create_topic(topic_metadata('/mission/test', 'std_msgs/msg/String'))
    message = String(data='mission')
    writer.write('/mission/test', serialize_message(message), timestamp_ns)


def _write_scene_bag(path: Path, timestamp_ns: int) -> None:
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    topics = (
        ('/mission/scene_cloud', 'sensor_msgs/msg/PointCloud2'),
        ('/mission/scene_markers', 'visualization_msgs/msg/MarkerArray'),
        ('/mission/scene_mesh', 'visualization_msgs/msg/Marker'),
        ('/mission/voxel_markers', 'visualization_msgs/msg/MarkerArray'),
    )
    for name, type_name in topics:
        writer.create_topic(topic_metadata(name, type_name))
    header = Header(stamp=ros_time(timestamp_ns), frame_id='map')
    cloud = PointCloud2(header=header)
    marker = Marker(header=header, id=1, type=Marker.POINTS)
    marker.points = [Point(x=1.0, y=2.0, z=3.0)]
    marker_array = MarkerArray(markers=[marker])
    messages = {
        '/mission/scene_cloud': cloud,
        '/mission/scene_markers': marker_array,
        '/mission/scene_mesh': marker,
        '/mission/voxel_markers': marker_array,
    }
    for topic, message in messages.items():
        writer.write(topic, serialize_message(message), timestamp_ns)


def test_integrated_replay_retimes_scene_and_connects_frames(tmp_path):
    mission_start = 1_700_000_000_000_000_000
    scene_start = 4_000_000_000
    mission_bag = tmp_path / 'mission'
    scene_bag = tmp_path / 'scene'
    output_bag = tmp_path / 'integrated'
    _write_mission_bag(mission_bag, mission_start)
    _write_scene_bag(scene_bag, scene_start)

    manifest = build_integrated_replay(
        mission_bag, scene_bag, output_bag
    )

    assert manifest['message_count'] == 6
    assert manifest['time_alignment']['scene_target_start_ns'] \
        == mission_start
    assert manifest['verification']['scene_headers_retimed'] is True
    assert manifest['verification']['map_to_odom_static_transform'] is True
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(output_bag), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    while reader.has_next():
        topic, serialized, timestamp = reader.read_next()
        if topic == '/mission/scene_cloud':
            message = deserialize_message(serialized, PointCloud2)
            value = message.header.stamp
            assert value.sec * 1_000_000_000 + value.nanosec == timestamp
