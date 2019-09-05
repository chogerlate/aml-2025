#!/usr/bin/python3
import os, sys

from kivy.uix.widget import Widget
from kivy.properties import NumericProperty, ReferenceListProperty, ObjectProperty, StringProperty
from kivy.vector import Vector
from kivy.logger import Logger
from kivy.core.window import Window

import random

from .Obstacle import Obstacle
from .Objective import Objective

class PySimbotMap(Widget):
    obstacles = ObjectProperty(None)
    objectives = ObjectProperty(None)
    iteration = NumericProperty(0)
    max_tick = NumericProperty(0)
    simulation_count = NumericProperty(0)

    # stats
    eat_count = NumericProperty(0)
    food_move_count = NumericProperty(0)
    score = NumericProperty(0)
    scoreStr = StringProperty("")

    def __init__(self, 
                robot_cls, 
                num_robots, 
                robot_start_pos, 
                max_tick,
                customfn_create_robots = None, 
                customfn_before_simulation = None,
                customfn_after_simulation = None,
                enable_wasd_control = False,
                simulation_forever = False,
                food_move_after_eat = True,
                **kwargs):
        super(PySimbotMap, self).__init__(**kwargs)
        self._keyboard = Window.request_keyboard(self._keyboard_closed, self)
        self._keyboard.bind(on_key_down=self._on_keyboard_down)
        
        self.obstacles = Obstacle()
        self.objectives = Objective()
        self.add_widget(self.obstacles)
        self.add_widget(self.objectives)

        self.robot_cls = robot_cls
        self.num_robots = num_robots
        self.robot_start_pos = robot_start_pos
        self.max_tick = max_tick
        self.customfn_create_robots = customfn_create_robots
        self.customfn_before_simulation = customfn_before_simulation
        self.customfn_after_simulation = customfn_after_simulation
        self.enable_wasd_control = enable_wasd_control
        self.food_move_after_eat = food_move_after_eat
        self.simulation_forever = simulation_forever

    def _create_robots(self):
        return self.customfn_create_robots() if self.customfn_create_robots else [self.robot_cls() for _ in range(self.num_robots)]

    def _before_simulation(self, map):
        if self.customfn_before_simulation:
            self.customfn_before_simulation(map)

    def _after_simulation(self, map):
        if self.customfn_after_simulation:
            self.customfn_after_simulation(map)

    def _add_robots_to_map(self, robots):
        for r in self.robots:
            r.pos = self.robot_start_pos
            r._sm = self
            self.add_widget(r)

    def _remove_all_robots_from_map(self):
        for r in self.robots:
            self.remove_widget(r)
        self.robots = []

    def _reset_stats(self):
        self.eat_count = 0
        self.food_move_count = 0
        self.score = 0
        if self.food_move_after_eat:
            self.scoreStr = str(self.score) + " %"
        else:
            self.scoreStr = str(self.score)

    def process(self, dt):
        if self.iteration == 0:
            self._reset_stats()
            self.robots = self._create_robots()
            self._add_robots_to_map(self.robots)
            self._before_simulation(self)
            self.simulation_count += 1
            Logger.debug('Map: Start Simulation')

        if self.iteration < self.max_tick:
            self.iteration += 1
            Logger.debug('Map: Start Iteration')
            for robot in self.robots:
                robot.update()
            Logger.debug('Map: End Iteration: {}'.format(self.iteration))
            if self.iteration == self.max_tick:
                self._after_simulation(self)
                Logger.debug('Map: End Simulation: {}'.format(self.simulation_count))
                if self.simulation_forever:
                    self._remove_all_robots_from_map()
                    self.iteration = 0
    
    def _keyboard_closed(self):
        self._keyboard.unbind(on_key_down=self._on_keyboard_down)
        self._keyboard = None
    
    def _on_keyboard_down(self, keyboard, keycode, text, modifiers):
        if self.iteration >= self.max_tick:
            return
        if keycode[1] == 'n':
            for obj in self.objectives.get_objectives():
                self.change_objective_pos(obj)
                self.food_move_count += 1
                self.score = int(self.eat_count * 100 / self.food_move_count)
        elif keycode[1] == 'w' and self.enable_wasd_control:
            for r in self.robots:
                r.move(5)
        elif keycode[1] == 'a' and self.enable_wasd_control:
            for r in self.robots:
                r.turn(-5)
        elif keycode[1] == 'd' and self.enable_wasd_control:
            for r in self.robots:
                r.turn(5)
        elif keycode[1] == 's' and self.enable_wasd_control:
            for r in self.robots:
                r.move(-5)

    def on_robot_eat(self, robot, obj):
        self.eat_count += 1
        if self.food_move_after_eat:
            self.food_move_count += 1
            self.change_objective_pos(obj)
            self.score = int(self.eat_count * 100 / self.food_move_count)
            self.scoreStr = str(self.score) + " %"
        else:
            self.score += 5
            self.scoreStr = str(self.score)

    def change_objective_pos(self, obj, pos=None):
        if pos is None:
            new_pos = Vector(random.randrange(self.playground.size[0]), random.randrange(self.playground.size[1]))
            while not self.is_objective_pos_valid(obj, new_pos):
                new_pos = Vector(random.randrange(self.playground.size[0]), random.randrange(self.playground.size[1]))
            obj.pos = new_pos
        else:        
            obj.pos = pos

    def is_objective_pos_valid(self, obj, pos):
        # check wall
        if pos.x < 10 or pos.x > self.size[0] - 210 - obj.size[0]:
            return False
        if pos.y < 10 or pos.y > self.size[1] -  10 - obj.size[1]:
            return False

        # check obstracles
        for obs in self.obstacles.get_obstacles():
            if (obs.pos[0] < pos.x < obs.pos[0] + obs.size[0] or obs.pos[0] < pos.x + obj.size[0] < obs.pos[0] + obs.size[0])\
                and (obs.pos[1] < pos.y < obs.pos[1] + obs.size[1] or obs.pos[1] < pos.y + obj.size[1] < obs.pos[1] + obs.size[1]):
                return False
        return True
