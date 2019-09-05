#!/usr/bin/python3

from pysimbotlib.App import PySimbotApp
from pysimbotlib.Robot import Robot
from kivy.logger import Logger
from kivy.config import Config

# Force the program to show user's log only for "info" level or more. The info log will be disabled.
Config.set('kivy', 'log_level', 'info')

# update robot every 0.33 seconds (3 frames per sec)
REFRESH_INTERVAL = 1/3

class MyRobot(Robot):
    
    def update(self):
        Logger.info("Smell Angle: {0}".format(self.smell()))
        Logger.info("Distance: {0}".format(self.distance()))

if __name__ == '__main__':
    app = PySimbotApp(robot_cls=MyRobot, num_robots=1, interval=REFRESH_INTERVAL, enable_wasd_control=True)
    app.run()