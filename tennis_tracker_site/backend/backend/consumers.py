from channels.generic.websocket import AsyncWebsocketConsumer
import json

# Update from SQL library
class EntryConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.channel_layer.group_add('entries', self.channel_name)
        await self.accept()
    async def new_entry(self,event):
        await self.send(text_data=json.dumps(event['data']))