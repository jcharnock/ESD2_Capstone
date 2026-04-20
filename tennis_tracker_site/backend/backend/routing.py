from django.urls import re_path
from .consumers import EntryConsumer

websocket_urlpatterns=[re_path(r'ws/entries/$', EntryConsumer.as_asgi())]