from rest_framework.generics import ListAPIView
from .models import TennisEntry
from .serializers import TennisEntrySerializer

# Read-only API for entries
class TennisEntryListView(ListAPIView):
    queryset = TennisEntry.objects.all()
    serializer_class = TennisEntrySerializer