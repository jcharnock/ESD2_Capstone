# app.py
# Flask backend where EACH ENTRY is its own MySQL table:
# serve_no_1, serve_no_2, serve_no_3 ...
# Only returns NEW tables not previously sent.

from flask import Flask, jsonify
import mysql.connector
from mysql.connector import Error

app = Flask(__name__)


# MySQL config
# DO NOT CHANGE USER AND PASSWORD
# IP configured for local pc
DB_CONFIG = {
    "host": "10.8.86.8",
    "port": 3306,
    "user": "flaskuser",
    "password": "tennistracker123",
    "database": "volleys"
}

# track table numbers
last_seen_table = 0


# connect to mySQL
def get_connection():
    return mysql.connector.connect(**DB_CONFIG)


# retrieve the new entries from SQL
def get_new_entries():
    global last_seen_table
    results = []

    try:
        conn = get_connection()
        cursor = conn.cursor(dictionary=True)

        # grab tables matching serve_no_#
        cursor.execute("SHOW TABLES LIKE 'serve_no_%'")
        tables = [row[f"Tables_in_volleys (serve_no_%)"] for row in cursor.fetchall()]

        # Extract numbers + sort
        numbered_tables = []

        for table in tables:
            try:
                num = int(table.replace("serve_no_", ""))
                if num > last_seen_table:
                    numbered_tables.append((num, table))
            except:
                pass

        numbered_tables.sort()

        # pull from new tables
        for num, table in numbered_tables:

            cursor.execute(f"SELECT * FROM {table}")
            rows = cursor.fetchall()

            # divvy entries into rows fro front end
            for entry in rows:
                entry["serve_number"] = num
                entry["table_name"] = table
                results.append(entry)

            # Update tracker
            last_seen_table = num

        cursor.close()
        conn.close()

    except Error as e:
        print("MySQL Error:", e)

    return results


# api routing
@app.route("/get_new_entries", methods=["GET"])
def get_entries():
    return jsonify(get_new_entries())

# run app
if __name__ == "__main__":
    app.run(debug=True, port=5000)