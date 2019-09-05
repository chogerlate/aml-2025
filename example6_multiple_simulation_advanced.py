#!/usr/bin/python3

from pysimbotlib.App import PySimbotApp
from pysimbotlib.PySimbotMap import PySimbotMap
from pysimbotlib.Robot import Robot
from kivy.logger import Logger

from kivy.config import Config
# Force the program to show user's log only for "info" level or more. The info log will be disabled.
Config.set('kivy', 'log_level', 'info')

import random

class RandomWalkRobot(Robot):

    def update(self):
        r = random.randint(0, 3)
        self.move(5)
        if r == 1:
            self.turn(15)
        elif r == 2:
            self.turn(-15)

def before_sim(simbot_map: PySimbotMap):
    Logger.info("Simulation: Before simulation.")
    Logger.info("Simulation: You can now do something with map objects or robots")
    for r in simbot_map.robots:
        r.pos = (400, 30)
        r.set_color(random.random(), random.random(), random.random())

def after_sim(simbot_map: PySimbotMap):
    Logger.info("Simulation: End simulation. Robot[0] is at {0}".format(simbot_map.robots[0].pos))
    Logger.info("Simulation: Score = {0}".format(simbot_map.score))
    for r in simbot_map.robots:
        Logger.info("Simulation: robot pos = {0}".format(r.pos))
        Logger.info("Simulation: robot eat_count = {0}".format(r.eat_count))

if __name__ == '__main__':
    app = PySimbotApp(robot_cls=RandomWalkRobot,
                        num_robots=5,
                        max_tick=400, 
                        simulation_forever=True,
                        customfn_before_simulation=before_sim,
                        customfn_after_simulation=after_sim,
                        food_move_after_eat=False)
    app.run()