#!/usr/bin/python3

import os, sys
import math
import random

from kivy.uix.widget import Widget
from kivy.properties import NumericProperty,\
                            ReferenceListProperty,\
                            ObjectProperty
from kivy.vector import Vector
from kivy.logger import Logger
from . import PySimbotMap

DEFAULT_DISTANCE_ANGLE = list(range(0, 360, 45))

class Robot(Widget):
    # Facing 0 degree direction
    _sm = ObjectProperty(None)
    _color_r = NumericProperty(0)
    _color_g = NumericProperty(0)
    _color_b = NumericProperty(0)
    _color_a = NumericProperty(0)
    _direction = NumericProperty(0)
    color = ReferenceListProperty(_color_r, _color_g, _color_b, _color_a)
    eat_count = NumericProperty(0)
    collision_count = NumericProperty(0)

    just_eat = False

    @property
    def _obstacles(self):
        if self._sm and hasattr(self._sm, 'obstacles'):
            return self._sm.obstacles.get_obstacles()
        return [] 
    
    @property
    def _objectives(self):
        if self._sm and hasattr(self._sm, 'objectives'):
            return self._sm.objectives.get_objectives()
        return []
    
    def _isValidPosition(self, p):
        # Check for outside wall
        if (p[0] < self.parent.x) or (p[0] > self.parent.x + self.parent.width) or (p[1] < self.parent.y) or (p[1] > self.parent.y + self.parent.height):
            return False
        # Check obstacles
        for obs in self._obstacles:
            if p[0] < obs.x or p[0] > obs.x + obs.width or p[1] < obs.y or p[1] > obs.y + obs.height:
                continue
            return False
        return True

    def _distance(self, angle):
        rad_angle = math.radians((360-(self._direction+angle))%360)
        unit_x = math.cos(rad_angle)
        unit_y = math.sin(rad_angle)
        # Start position
        surf = Vector(self.width / 2.0 * unit_x, self.height/2.0 * unit_y) + self.center
        for i in range(0, 100):
            new_i = Vector(i * unit_x, i * unit_y) + surf
            if self._isValidPosition(new_i):
                continue
            return i
        return 100

    def _isValidMove(self, next_position):
        center = Vector(self.width / 2, self.height / 2) + next_position
        for angle in range(0, 360, 4):
            rad_angle = math.radians((360-(self._direction+angle))%360)
            unit_x = math.cos(rad_angle)
            unit_y = math.sin(rad_angle)
            surf = Vector(self.width / 2.0 * unit_x, self.height/2.0 * unit_y) + center
            if not self._isValidPosition(surf):
                return False
        return True

    def _get_overlap_objective(self):
        for obj in self._objectives:
            if (self.pos[0] < obj.x and self.pos[0] + self.width < obj.x) or \
                (self.pos[0] > obj.x + obj.width and self.pos[0] + self.width > obj.x + obj.width) or \
                (self.pos[1] < obj.y and self.pos[1] + self.height < obj.y) or \
                (self.pos[1] > obj.y + obj.height and self.pos[1] + self.height > obj.y + obj.height):
                continue
            return obj
        return None
        
    def set_color(self, r, g, b, a=1):
        self._color_r = r
        self._color_g = g
        self._color_b = b
        self._color_a = a
    
    def distance(self):
        r = []
        for angle in DEFAULT_DISTANCE_ANGLE:
            r.append(self._distance(angle))
        return r
    
    def smell(self, index=0):
        if index >= 0 and index < len(self._objectives):
            # Get angle
            obj = self._objectives[index]
            dvx = self.center_x - obj.center_x
            dvy = self.center_y - obj.center_y
            rad = math.atan2(dvy, dvx)
            deg = ((540 - (math.degrees(rad) + self._direction))%360)
            if(deg <= 180):
                return deg
            else:
                return deg - 360
        return -1
    
    def turn(self, degree: float = 1):
        self._direction = (self._direction + degree + 360) % 360

    def move(self, step: int = 1):
        if step >= 0:
            rad_angle = math.radians((360-self._direction)%360)
            step = int(step)
        else:
            rad_angle = math.radians((540-self._direction)%360)
            step = int(-step)
        dx = math.cos(rad_angle)
        dy = math.sin(rad_angle)
        for _ in range(step):
            next_position = Vector(dx, dy) + self.pos
            # If can move
            if not self._isValidMove(next_position):
                self.collision_count += 1
                break
            self.pos = next_position
        
        obj = self._get_overlap_objective()
        if not obj:
            self.just_eat = False
        elif obj and not self.just_eat:
            Logger.debug('Robot: Eat Objective at [{}, {}]'.format(obj.pos[0], obj.pos[1]))
            self._sm.on_robot_eat(self, obj)
            self.eat_count += 1
            self.just_eat = True

    def update(self):
        pass