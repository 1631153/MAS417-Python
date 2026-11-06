import sqlite3

def get_connection():
    conn = sqlite3.connect('helpdesk.db')
    conn.row_factory = sqlite3.Row
    return conn

def create_tables():
    conn = get_connection()
    cursor = conn.cursor()
    
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS tickets (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            name TEXT NOT NULL,
            swing_period REAL NOT NULL,
            max_size REAL NOT NULL,
            axle_diameter REAL NOT NULL,
            infill INTEGER NOT NULL,
            walls INTEGER NOT NULL,
            filament_base INTEGER,
            filament_stands INTEGER,
            filament_arm INTEGER,
            filament_weight INTEGER,
            filament_axle INTEGER,
            creation_date TEXT,
            status TEXT NOT NULL DEFAULT 'open'
        )
    ''')
    
    conn.commit()
    conn.close()