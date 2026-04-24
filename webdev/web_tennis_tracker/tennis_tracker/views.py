from django.db import connection
from rest_framework.decorators import api_view
from rest_framework.response import Response
from .utils import get_last_seen, set_last_seen

# Read-only API for entries
@api_view(['GET'])
def get_new_serves(request):

    new_rows = []

    with connection.cursor() as cursor:

        cursor.execute("SHOW TABLES LIKE 'serve_no_%'")
        tables = [t[0] for t in cursor.fetchall()]

        for table in tables:

            last_seen = get_last_seen(table)

            cursor.execute(f"""
                SELECT entry_number, timestamp, x_coordinate, y_coordinate, ruling
                FROM {table}
                WHERE entry_number > %s
                ORDER BY entry_number ASC
            """, [last_seen])

            rows = cursor.fetchall()

            # get data into views
            for r in rows:
                new_rows.append({
                    "table": table,
                    "entry_number": r[0],
                    "timestamp": r[1],
                    "x_coordinate": r[2],
                    "y_coordinate": r[3],
                    "ruling": r[4],
                })

                # update cursor
                set_last_seen(table, r[0])

    return Response(new_rows)