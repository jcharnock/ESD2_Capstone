from rest_framework import serializers
from .models import TennisEntry

# Converts model rows into JSON for front end
class TennisEntrySerializer(serializers.ModelSerializer):
    class Meta:
        model = TennisEntry
        fields = '__all__'