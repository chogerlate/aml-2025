#!/usr/bin/python3

from pysimbotlib.App import PySimbotApp
from pysimbotlib.PySimbotMap import PySimbotMap
from pysimbotlib.Robot import Robot
from kivy.logger import Logger

from kivy.config import Config
# Force the program to show user's log only for "info" level or more. The info log will be disabled.
Config.set('kivy', 'log_level', 'info')

import random

if __name__ == '__main__':
    # possible theme value: ["default", "light", "dark"]
    app = PySimbotApp(theme="light")
    app.run()