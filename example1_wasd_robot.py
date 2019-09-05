#!/usr/bin/python3

from pysimbotlib.App import PySimbotApp
from kivy.config import Config

# Force the program to show user's log only for "debug" level or more. The info log will be disabled.
Config.set('kivy', 'log_level', 'debug')

if __name__ == '__main__':
    app = PySimbotApp(enable_wasd_control=True)
    app.run()