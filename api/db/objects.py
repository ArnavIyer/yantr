import sqlite3
import os
from dataclasses import dataclass

import numpy as np


@dataclass
class RegisteredObject:
    id: int
    map: str
    name: str
    embedding: np.ndarray  # 1152-dim float32
    cx: float
    cy: float
    cz: float
    point_cloud: np.ndarray  # flat float32 array
    created_at: str


def _row_to_object(row: sqlite3.Row) -> RegisteredObject:
    return RegisteredObject(
        id=row["id"],
        map=row["map"],
        name=row["name"],
        embedding=np.frombuffer(row["embedding"], dtype=np.float32) if row["embedding"] else np.array([], dtype=np.float32),
        cx=row["cx"],
        cy=row["cy"],
        cz=row["cz"],
        point_cloud=np.frombuffer(row["point_cloud"], dtype=np.float32) if row["point_cloud"] else np.array([], dtype=np.float32),
        created_at=row["created_at"],
    )


class ObjectStore:
    def __init__(self, db_path: str):
        self.conn = sqlite3.connect(db_path)
        self.conn.row_factory = sqlite3.Row
        schema_path = os.path.join(os.path.dirname(__file__), "schema.sql")
        with open(schema_path) as f:
            self.conn.executescript(f.read())

    def insert(self, map: str, name: str, embedding: np.ndarray, cx: float, cy: float, cz: float, point_cloud: np.ndarray) -> int:
        cur = self.conn.execute(
            "INSERT INTO object (map, name, embedding, cx, cy, cz, point_cloud) VALUES (?, ?, ?, ?, ?, ?, ?)",
            (map, name, embedding.astype(np.float32).tobytes(), cx, cy, cz, point_cloud.astype(np.float32).tobytes()),
        )
        self.conn.commit()
        return cur.lastrowid

    def search_by_embedding(self, embedding: np.ndarray, threshold: float) -> list[RegisteredObject]:
        rows = self.conn.execute("SELECT * FROM object").fetchall()
        query = embedding.astype(np.float32).flatten()
        query_norm = np.linalg.norm(query)
        if query_norm == 0:
            return []

        results = []
        for row in rows:
            obj = _row_to_object(row)
            if obj.embedding.size == 0:
                continue
            stored_norm = np.linalg.norm(obj.embedding)
            if stored_norm == 0:
                continue
            similarity = np.dot(query, obj.embedding) / (query_norm * stored_norm)
            if similarity >= threshold:
                results.append((similarity, obj))

        results.sort(key=lambda x: x[0], reverse=True)
        return [obj for _, obj in results[:3]]

    def search_by_location(self, x: float, y: float, z: float, max_distance: float, limit: int = 3) -> list[RegisteredObject]:
        rows = self.conn.execute("SELECT * FROM object").fetchall()
        query = np.array([x, y, z])

        results = []
        for row in rows:
            obj = _row_to_object(row)
            centroid = np.array([obj.cx, obj.cy, obj.cz])
            dist = np.linalg.norm(query - centroid)
            if dist <= max_distance:
                results.append((dist, obj))

        results.sort(key=lambda x: x[0])
        return [obj for _, obj in results[:limit]]
