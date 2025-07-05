import sqlite3

conn = sqlite3.connect("tags.db")
cursor = conn.cursor()

# Ensure the table exists
cursor.execute("""
    CREATE TABLE IF NOT EXISTS tags (
        id INTEGER PRIMARY KEY,
        seed_type TEXT,
        x REAL,
        y REAL,
        z REAL
    )
""")

# Insert or update tag info for ID 584
cursor.execute("INSERT OR REPLACE INTO tags (id, seed_type, x, y, z) VALUES (?, ?, ?, ?, ?)",
               (584, "Sunflower", 12.5, 45.3, 6.8))

conn.commit()
conn.close()

print("Tag 584 added successfully!")
