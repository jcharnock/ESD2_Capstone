# Database table for each tennis shot entry
from django.db import models

# Stores each tennis call imported from SQL/database feed
class TennisEntry(models.Model):
    # Sequential entry number shown in list
    entry_number = models.IntegerField()

    # Match clock time or system time string
    timestamp = models.CharField(max_length=50)

    # Ball location coordinates
    x_coordinate = models.IntegerField()
    y_coordinate = models.IntegerField()

    # IN / OUT / NET text field
    ruling = models.CharField(max_length=20, default='IN')

    # Image path or url to court image
    court_image = models.TextField(blank=True, null=True)

    # newest entries displayed first
    class Meta:
        ordering = ['-id']  

    def __str__(self):
        return f'Entry {self.entry_number}' 