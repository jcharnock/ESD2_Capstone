from django.core.cache import cache

# basic functions to prevent repreat entries from being added
def get_last_seen(table):
    return cache.get(f"last_seen_{table}", 0)

def set_last_seen(table, value):
    cache.set(f"last_seen_{table}", value, timeout=None)