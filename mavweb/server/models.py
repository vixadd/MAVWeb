""" server.models
The server model that allows us to structure data that
is stored.
    Author: davidkroell
"""
from django.db import models


class Telemetry(models.Model):
    """
    Telemetry storage module.
    """
    longitude = models.FloatField(default=0.0)
    latitude  = models.FloatField(default=0.0)
