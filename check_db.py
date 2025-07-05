import sqlite3

conn = sqlite3.connect("tags.db")
cursor = conn.cursor()

# Fetch all stored tag data
cursor.execute("SELECT * FROM tags")
data = cursor.fetchall()

if not data:
    print("Database is empty! Ensure you added the tag ID.")
else:
    for row in data:
        print(row)  # Print each stored tag entry

conn.close()
