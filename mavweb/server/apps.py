from django.apps import AppConfig
from mavweb.mavlink_arbiter.main import Main


class ServerConfig(AppConfig):
    """
    Django Server configurations.
    """
    name = 'server'


class MavlinkArbiter(AppConfig):
    """
    Mavlink arbiter used for application pairing with mavlink functions.
    """
    name = 'mavlink_arbiter'
    arbiter_module = Main()
