CREATE TABLE IF NOT EXISTS object (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    map           TEXT NOT NULL,
    name          TEXT NOT NULL,
    embedding     BLOB,           -- float32 bytes, 1152-dim (google/siglip2-so400m-patch16-naflex)
    cx            REAL,           -- centroid x
    cy            REAL,           -- centroid y
    cz            REAL,           -- centroid z
    point_cloud   BLOB,           -- raw point cloud blob (e.g. PCD/PLY bytes)
    created_at    TEXT DEFAULT (strftime('%Y-%m-%dT%H:%M:%fZ', 'now'))
);

CREATE INDEX IF NOT EXISTS map_idx ON object (map);
