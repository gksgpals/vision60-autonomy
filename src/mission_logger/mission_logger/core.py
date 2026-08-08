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

from dataclasses import dataclass
from hashlib import sha256
import json
import sqlite3
from threading import Lock
from typing import Callable, Dict, Optional, Protocol


@dataclass(frozen=True)
class MissionItem:
    item_id: str
    mission_id: str
    category: str
    timestamp_ns: int
    payload_json: str
    checksum: str
    attempts: int


@dataclass(frozen=True)
class SyncResult:
    success: bool
    synchronized_items: int
    failed_items: int
    remaining_items: int


class SyncTransport(Protocol):
    def send(self, item: MissionItem) -> bool:
        """Return true only after the receiver acknowledges the item."""


def canonical_json(payload: Dict) -> str:
    return json.dumps(
        payload,
        ensure_ascii=False,
        sort_keys=True,
        separators=(',', ':'),
    )


def item_checksum(
    mission_id: str,
    category: str,
    timestamp_ns: int,
    payload_json: str,
) -> str:
    content = (
        f'{mission_id}\n{category}\n{timestamp_ns}\n{payload_json}'
    )
    return sha256(content.encode('utf-8')).hexdigest()


class MissionStore:
    def __init__(self, database_path: str) -> None:
        self._connection = sqlite3.connect(
            database_path,
            check_same_thread=False,
        )
        self._connection.row_factory = sqlite3.Row
        self._lock = Lock()
        with self._lock:
            self._connection.execute('PRAGMA journal_mode=WAL')
            self._connection.execute('PRAGMA synchronous=FULL')
        self._create_schema()

    def _create_schema(self) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                '''
                CREATE TABLE IF NOT EXISTS mission_items (
                    item_id TEXT PRIMARY KEY,
                    mission_id TEXT NOT NULL,
                    category TEXT NOT NULL,
                    timestamp_ns INTEGER NOT NULL,
                    payload_json TEXT NOT NULL,
                    checksum TEXT NOT NULL,
                    sync_state TEXT NOT NULL DEFAULT 'pending',
                    attempts INTEGER NOT NULL DEFAULT 0,
                    last_error TEXT NOT NULL DEFAULT '',
                    synchronized_at TEXT,
                    UNIQUE(mission_id, checksum)
                )
                '''
            )
            self._connection.execute(
                '''
                CREATE INDEX IF NOT EXISTS idx_mission_pending
                ON mission_items(mission_id, sync_state, timestamp_ns)
                '''
            )

    def enqueue(
        self,
        mission_id: str,
        category: str,
        timestamp_ns: int,
        payload: Dict,
        item_id: str = '',
    ) -> bool:
        if not mission_id:
            raise ValueError('mission_id must not be empty')
        if not category:
            raise ValueError('category must not be empty')
        if timestamp_ns < 0:
            raise ValueError('timestamp_ns must be non-negative')

        payload_json = canonical_json(payload)
        checksum = item_checksum(
            mission_id,
            category,
            timestamp_ns,
            payload_json,
        )
        stable_id = item_id or checksum
        with self._lock, self._connection:
            cursor = self._connection.execute(
                '''
                INSERT OR IGNORE INTO mission_items (
                    item_id, mission_id, category, timestamp_ns,
                    payload_json, checksum
                ) VALUES (?, ?, ?, ?, ?, ?)
                ''',
                (
                    stable_id,
                    mission_id,
                    category,
                    timestamp_ns,
                    payload_json,
                    checksum,
                ),
            )
        return cursor.rowcount == 1

    def pending(
        self,
        mission_id: str,
        limit: int = 100,
    ) -> list[MissionItem]:
        with self._lock:
            rows = self._connection.execute(
                '''
                SELECT item_id, mission_id, category, timestamp_ns,
                       payload_json, checksum, attempts
                FROM mission_items
                WHERE mission_id = ? AND sync_state != 'synced'
                ORDER BY timestamp_ns ASC, item_id ASC
                LIMIT ?
                ''',
                (mission_id, limit),
            ).fetchall()
        return [MissionItem(**dict(row)) for row in rows]

    def mark_synced(self, item_id: str) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                '''
                UPDATE mission_items
                SET sync_state = 'synced',
                    attempts = attempts + 1,
                    last_error = '',
                    synchronized_at = CURRENT_TIMESTAMP
                WHERE item_id = ?
                ''',
                (item_id,),
            )

    def mark_failed(self, item_id: str, error: str) -> None:
        with self._lock, self._connection:
            self._connection.execute(
                '''
                UPDATE mission_items
                SET sync_state = 'failed',
                    attempts = attempts + 1,
                    last_error = ?
                WHERE item_id = ?
                ''',
                (error, item_id),
            )

    def counts(self, mission_id: str) -> Dict[str, int]:
        result = {'pending': 0, 'failed': 0, 'synced': 0}
        with self._lock:
            rows = self._connection.execute(
                '''
                SELECT sync_state, COUNT(*) AS count
                FROM mission_items
                WHERE mission_id = ?
                GROUP BY sync_state
                ''',
                (mission_id,),
            ).fetchall()
        for row in rows:
            result[row['sync_state']] = row['count']
        return result

    def close(self) -> None:
        with self._lock:
            self._connection.close()


class MissionSynchronizer:
    def __init__(
        self,
        store: MissionStore,
        transport: SyncTransport,
        batch_size: int = 100,
    ) -> None:
        self.store = store
        self.transport = transport
        self.batch_size = batch_size

    def synchronize(
        self,
        mission_id: str,
        progress_callback: Optional[
            Callable[[MissionItem, int, int], None]
        ] = None,
    ) -> SyncResult:
        items = self.store.pending(mission_id, self.batch_size)
        synchronized = 0
        failed = 0
        total = len(items)
        for index, item in enumerate(items, start=1):
            try:
                acknowledged = self.transport.send(item)
            except Exception as error:
                acknowledged = False
                failure = str(error)
            else:
                failure = 'receiver did not acknowledge item'

            if acknowledged:
                self.store.mark_synced(item.item_id)
                synchronized += 1
            else:
                self.store.mark_failed(item.item_id, failure)
                failed += 1

            if progress_callback is not None:
                progress_callback(item, index, total)

        counts = self.store.counts(mission_id)
        remaining = counts['pending'] + counts['failed']
        return SyncResult(
            success=failed == 0 and remaining == 0,
            synchronized_items=synchronized,
            failed_items=failed,
            remaining_items=remaining,
        )


class MockSyncTransport:
    def __init__(self, fail_after_items: int = -1) -> None:
        self.fail_after_items = fail_after_items
        self.received: Dict[str, str] = {}
        self._send_count = 0

    def send(self, item: MissionItem) -> bool:
        self._send_count += 1
        if (
            self.fail_after_items >= 0
            and self._send_count > self.fail_after_items
        ):
            return False

        previous_checksum = self.received.get(item.item_id)
        if previous_checksum is not None:
            return previous_checksum == item.checksum
        self.received[item.item_id] = item.checksum
        return True
