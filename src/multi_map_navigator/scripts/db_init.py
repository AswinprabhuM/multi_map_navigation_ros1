import sqlite3

# Create the database
conn = sqlite3.connect('../my_wormholes.db')
c = conn.cursor()

# Create wormhole table
c.execute('''
CREATE TABLE wormholes (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    source_map TEXT NOT NULL,
    target_map TEXT NOT NULL,
    source_x REAL NOT NULL,
    source_y REAL NOT NULL,
    source_yaw REAL DEFAULT 0,
    target_x REAL NOT NULL,
    target_y REAL NOT NULL,
    target_yaw REAL DEFAULT 0
)
''')

# Insert sample wormhole from room_a to room_b
c.execute('''
INSERT INTO wormholes (source_map, target_map, source_x, source_y, source_yaw, target_x, target_y, target_yaw)
VALUES ("room_a", "room_b", 2.045433, -2.001620, 3.116040, 1.709460, -1.071437, -3.098331)
''')

# (Optional) Add reverse wormhole from room_b to room_a
c.execute('''
INSERT INTO wormholes (source_map, target_map, source_x, source_y, source_yaw, target_x, target_y, target_yaw)
VALUES ("room_b", "room_a", 1.709460, -1.071437, -3.098331, 2.045433, -2.001620, 3.116040)
''')

conn.commit()
conn.close()

print("Database my_wormholes.db created successfully.")

