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

import sqlite3

from mission_logger.core import (
    MissionStore,
    MissionSynchronizer,
    MockSyncTransport,
)


def test_duplicate_item_is_stored_once(tmp_path):
    store = MissionStore(str(tmp_path / 'mission.sqlite3'))
    payload = {'x': 1.0, 'event': 'link_lost'}

    assert store.enqueue(
        'mission_001', 'event', 100, payload, 'event_001'
    )
    assert not store.enqueue(
        'mission_001', 'event', 100, payload, 'event_001'
    )

    assert len(store.pending('mission_001')) == 1
    store.close()


def test_same_content_with_different_id_is_deduplicated(tmp_path):
    store = MissionStore(str(tmp_path / 'mission.sqlite3'))
    payload = {'state': 'lost'}

    assert store.enqueue(
        'mission_001', 'communication', 100, payload, 'sample_a'
    )
    assert not store.enqueue(
        'mission_001', 'communication', 100, payload, 'sample_b'
    )

    assert len(store.pending('mission_001')) == 1
    store.close()


def test_successful_sync_marks_items_and_does_not_resend(tmp_path):
    store = MissionStore(str(tmp_path / 'mission.sqlite3'))
    for index in range(3):
        store.enqueue(
            'mission_001',
            'odometry',
            index,
            {'x': float(index)},
        )
    transport = MockSyncTransport()
    synchronizer = MissionSynchronizer(store, transport)

    result = synchronizer.synchronize('mission_001')
    repeated = synchronizer.synchronize('mission_001')

    assert result.success
    assert result.synchronized_items == 3
    assert repeated.synchronized_items == 0
    assert len(transport.received) == 3
    store.close()


def test_failed_item_remains_retryable(tmp_path):
    store = MissionStore(str(tmp_path / 'mission.sqlite3'))
    store.enqueue('mission_001', 'event', 100, {'type': 'victim'})

    failed = MissionSynchronizer(
        store,
        MockSyncTransport(fail_after_items=0),
    ).synchronize('mission_001')
    retried = MissionSynchronizer(
        store,
        MockSyncTransport(),
    ).synchronize('mission_001')

    assert not failed.success
    assert failed.remaining_items == 1
    assert retried.success
    assert retried.synchronized_items == 1
    store.close()


def test_database_survives_reopen(tmp_path):
    path = str(tmp_path / 'mission.sqlite3')
    store = MissionStore(path)
    store.enqueue('mission_001', 'event', 100, {'type': 'hazard'})
    store.close()

    reopened = MissionStore(path)
    assert len(reopened.pending('mission_001')) == 1
    reopened.close()

    connection = sqlite3.connect(path)
    integrity = connection.execute('PRAGMA integrity_check').fetchone()[0]
    connection.close()
    assert integrity == 'ok'
