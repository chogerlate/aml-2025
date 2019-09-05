from typing import ClassVar
from kivy.config import Config
Config.set('graphics', 'resizable', '0') #0 being off 1 being on as in true/false
Config.set('input', 'mouse', 'mouse,multitouch_on_demand')

import os
import platform
from kivy.app import App
from kivy.logger import Logger
from kivy.lang import Builder
from kivy.clock import Clock
from kivy.core.window import Window

from .PySimbotMap import PySimbotMap
from .Scaler import Scaler

from .Robot import Robot

MAP_DIRECTORY = './maps'
DEFAULT_MAP_NAME = 'default_map.kv'
DEFAULT_MAP_PATH = os.path.join(MAP_DIRECTORY, DEFAULT_MAP_NAME)
ROBOT_START_POS = (20, 560)

class PySimbotApp(App):

    title = 'PySimbot (Created by Saran Khotsathian (Bird), Edited by Chattriya Jariyavajee (Jet) CPE25)'

    def __init__(self,
                robot_cls = Robot,
                num_robots = 1, 
                robot_start_pos = ROBOT_START_POS,
                map_path = DEFAULT_MAP_PATH, 
                interval = 1.0/60.0,
                max_tick = 4000,
                theme = 'default',
                customfn_create_robots = None,
                customfn_before_simulation = None,
                customfn_after_simulation = None,
                enable_wasd_control = False,
                simulation_forever = False,
                food_move_after_eat = True,
                **kwargs):

        super(PySimbotApp, self).__init__(**kwargs)
        Logger.info('Map Path: %s' % map_path)
        self.interval = interval

        Window.size = (900, 600)
        Builder.load_file(map_path)
        if theme == "default":
            Builder.load_file('pysimbotlib/widget/robot.kv')
            Builder.load_file('pysimbotlib/widget/simbotmap.kv')
        elif theme == "dark":
            Builder.load_file('pysimbotlib/widget_dark/robot.kv')
            Builder.load_file('pysimbotlib/widget_dark/simbotmap.kv')
        elif theme == "light":
            Builder.load_file('pysimbotlib/widget_light/robot.kv')
            Builder.load_file('pysimbotlib/widget_light/simbotmap.kv')

        self.simbotMap = PySimbotMap(robot_cls = robot_cls, 
                            num_robots = num_robots, 
                            robot_start_pos = robot_start_pos, 
                            max_tick = max_tick, 
                            customfn_create_robots = customfn_create_robots,
                            customfn_before_simulation = customfn_before_simulation,
                            customfn_after_simulation = customfn_after_simulation,
                            enable_wasd_control = enable_wasd_control,
                            simulation_forever = simulation_forever,
                            food_move_after_eat = food_move_after_eat)

    def build(self):
        Clock.schedule_interval(self.simbotMap.process, self.interval)

        if platform.system() == 'Darwin':
            self._scaler = Scaler(size=Window.size, scale=2)
            Window.add_widget(self._scaler)
            parent = self._scaler or Window
            parent.add_widget(self.simbotMap)
        else:
            return self.simbotMap
