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

import os
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from vision60_msgs.action import SynchronizeMission
from vision60_msgs.msg import (
    CommunicationState,
    MissionEvent,
    RecoveryEvent,
    RecoveryStatus,
    RobotSafetyState,
    SyncStatus,
)

from mission_logger.core import (
    MissionItem,
    MissionStore,
    MissionSynchronizer,
    MockSyncTransport,
)


class MissionLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_logger')
        self.declare_parameter('mission_id', 'mission_001')
        self.declare_parameter(
            'database_path',
            '~/.local/share/vision60/mission.sqlite3',
        )
        self.declare_parameter('record_all_data', False)
        self.declare_parameter('telemetry_period_s', 0.5)
        self.declare_parameter('sync_batch_size', 10000)
        self.declare_parameter('transport', 'mock')
        self.declare_parameter('mock_failure_after_items', -1)

        database_path = os.path.abspath(
            os.path.expanduser(
                str(self.get_parameter('database_path').value)
            )
        )
        database_directory = os.path.dirname(database_path)
        if database_directory:
            os.makedirs(database_directory, exist_ok=True)
        self._store = MissionStore(database_path)

        transport_name = str(self.get_parameter('transport').value)
        if transport_name != 'mock':
            raise RuntimeError(
                f'unsupported sync transport: {transport_name}'
            )
        self._transport = MockSyncTransport(
            int(
                self.get_parameter(
                    'mock_failure_after_items'
                ).value
            )
        )
        self._synchronizer = MissionSynchronizer(
            self._store,
            self._transport,
            int(self.get_parameter('sync_batch_size').value),
        )
        self._link_connected = True
        self._last_record_ns: dict[str, int] = {}

        self._sync_status_publisher = self.create_publisher(
            SyncStatus, '/mission/sync_status', 10
        )
        self.create_subscription(
            CommunicationState,
            '/communication/state',
            self._communication_callback,
            20,
        )
        self.create_subscription(
            Odometry,
            '/state/odometry',
            lambda message: self._record_telemetry(
                'odometry', message
            ),
            20,
        )
        self.create_subscription(
            RobotSafetyState,
            '/vision60/safety_state',
            lambda message: self._record_telemetry(
                'robot_safety', message
            ),
            10,
        )
        self.create_subscription(
            RecoveryStatus,
            '/communication/recovery_status',
            lambda message: self._record_message(
                'recovery_status', message
            ),
            10,
        )
        self.create_subscription(
            RecoveryEvent,
            '/communication/recovery_event',
            lambda message: self._record_message(
                'recovery_event', message
            ),
            10,
        )
        self.create_subscription(
            MissionEvent,
            '/mission/event',
            self._mission_event_callback,
            10,
        )
        self._action_server = ActionServer(
            self,
            SynchronizeMission,
            '/mission/synchronize',
            self._execute_sync,
        )
        self.get_logger().info(
            f'Mission Logger ready: {database_path}'
        )

    def _communication_callback(
        self,
        message: CommunicationState,
    ) -> None:
        self._link_connected = bool(message.connected)
        if self._should_record_telemetry():
            self._record_message('communication', message)

    def _mission_event_callback(self, message: MissionEvent) -> None:
        self._record_message(
            'mission_event',
            message,
            item_id=message.event_id,
        )

    def _record_telemetry(self, category: str, message) -> None:
        if not self._should_record_telemetry():
            return
        period_ns = int(
            float(self.get_parameter('telemetry_period_s').value)
            * 1e9
        )
        now_ns = self.get_clock().now().nanoseconds
        previous_ns = self._last_record_ns.get(category)
        if (
            previous_ns is not None
            and now_ns - previous_ns < period_ns
        ):
            return
        self._last_record_ns[category] = now_ns
        self._record_message(category, message)

    def _should_record_telemetry(self) -> bool:
        return (
            bool(self.get_parameter('record_all_data').value)
            or not self._link_connected
        )

    def _record_message(
        self,
        category: str,
        message,
        item_id: str = '',
    ) -> None:
        mission_id = (
            getattr(message, 'mission_id', '')
            or self.get_parameter('mission_id').value
        )
        timestamp_ns = self._message_timestamp_ns(message)
        payload = dict(message_to_ordereddict(message))
        inserted = self._store.enqueue(
            str(mission_id),
            category,
            timestamp_ns,
            payload,
            item_id,
        )
        if not inserted:
            self.get_logger().debug(
                f'Duplicate mission item ignored: {category}'
            )

    def _message_timestamp_ns(self, message) -> int:
        header = getattr(message, 'header', None)
        stamp = getattr(header, 'stamp', None)
        if stamp is not None:
            timestamp_ns = int(stamp.sec) * 1_000_000_000
            timestamp_ns += int(stamp.nanosec)
            if timestamp_ns > 0:
                return timestamp_ns
        return self.get_clock().now().nanoseconds

    def _execute_sync(self, goal_handle):
        mission_id = (
            goal_handle.request.mission_id
            or self.get_parameter('mission_id').value
        )
        before = self._store.counts(mission_id)
        total = before['pending'] + before['failed']
        self._publish_sync_status(
            mission_id,
            SyncStatus.IN_PROGRESS,
            total,
            0,
            0,
            'synchronization started',
        )

        synchronized_so_far = 0

        def publish_progress(
            item: MissionItem,
            index: int,
            item_count: int,
        ) -> None:
            nonlocal synchronized_so_far
            counts = self._store.counts(mission_id)
            synchronized_so_far = (
                counts['synced'] - before['synced']
            )
            feedback = SynchronizeMission.Feedback()
            feedback.progress_ratio = (
                float(index) / float(item_count)
                if item_count
                else 1.0
            )
            feedback.remaining_items = max(item_count - index, 0)
            feedback.current_item = item.item_id
            goal_handle.publish_feedback(feedback)

        result = self._synchronizer.synchronize(
            mission_id,
            publish_progress,
        )
        response = SynchronizeMission.Result()
        response.success = result.success
        response.synchronized_items = result.synchronized_items
        response.failed_items = result.failed_items
        response.message = (
            'all pending mission data synchronized'
            if result.success
            else (
                f'{result.remaining_items} mission items remain '
                'unsynchronized'
            )
        )

        if result.success:
            goal_handle.succeed()
            status_state = SyncStatus.COMPLETE
        else:
            goal_handle.abort()
            status_state = SyncStatus.FAILED
        self._publish_sync_status(
            mission_id,
            status_state,
            total,
            result.synchronized_items,
            result.failed_items,
            response.message,
        )
        return response

    def _publish_sync_status(
        self,
        mission_id: str,
        state: int,
        total: int,
        synchronized: int,
        failed: int,
        detail: str,
    ) -> None:
        status = SyncStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = 'map'
        status.mission_id = mission_id
        status.state = state
        status.total_items = total
        status.synchronized_items = synchronized
        status.failed_items = failed
        status.progress_ratio = (
            float(synchronized + failed) / float(total)
            if total
            else 1.0
        )
        status.detail = detail
        self._sync_status_publisher.publish(status)

    def destroy_node(self) -> bool:
        self._action_server.destroy()
        self._store.close()
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MissionLoggerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
