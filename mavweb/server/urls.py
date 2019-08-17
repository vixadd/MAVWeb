""" mavweb.urls
Mavweb url page that is used to configure url denoatations
for the web application.

This includes the core REST Framework that is being built on.
"""

from django.contrib import admin
from django.urls import include, path
from mavweb.server import views

urlpatterns = [
    path('', views.index, name='index')
]
