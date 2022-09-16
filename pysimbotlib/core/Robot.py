#!/usr/bin/python3

import os, sys
import math
import random

from itertools import chain

from kivy.uix.widget import Widget
from kivy.properties import NumericProperty, ReferenceListProperty, ObjectProperty
from kivy.logger import Logger
from typing import Sequence, Tuple
from .Objective import Objective
from .Util import Util
from .config import SIMBOTMAP_SIZE, SIMBOTMAP_BBOX , ROBOT_DISTANCE_ANGLES, ROBOT_MAX_SENSOR_DISTANCE

class Robot(Widget):

    # Facing 0 degree direction
    _sm = None
    _direction = NumericProperty(0)
    
    _color_r = NumericProperty(0)
    _color_g = NumericProperty(0)
    _color_b = NumericProperty(0)
    _color_a = NumericProperty(0)

    color = ReferenceListProperty(_color_r, _color_g, _color_b, _color_a)
    
    eat_count: int = 0
    collision_count: int = 0
    just_eat: bool = False
    stuck: bool = False

    @staticmethod
    def distance_to_line_generators(surf: Util.Point2D, outside_bot: Util.Point2D, bounding_lines):
        # overlapping_bounding_lines = obstacle_bounding_lines
        for line in bounding_lines:
            intersection = Util.line_segment_intersect(surf, outside_bot, line[0], line[1])
            yield (Util.distance(surf, intersection) if intersection else ROBOT_MAX_SENSOR_DISTANCE)

    @staticmethod
    def distance_to_robot_generators(surf: Util.Point2D, outside_bot: Util.Point2D, robots):
        for r in robots:
            intersection = Util.line_segment_circle_intersect(surf, outside_bot, r.center, r.width / 2)
            near_intersection = intersection[0]
            yield (Util.distance(surf, near_intersection) if near_intersection else ROBOT_MAX_SENSOR_DISTANCE)
        yield ROBOT_MAX_SENSOR_DISTANCE

    def _distance(self, angle: float) -> float:
        rad_angle = math.radians(-(self._direction+angle) % 360)
        unit_x = math.cos(rad_angle)
        unit_y = math.sin(rad_angle)

        surf = (self.center_x + self.width / 2.0 * unit_x, 
                self.center_y + self.height / 2.0 * unit_y)
        ROI = ( min(surf[0], surf[0] + ROBOT_MAX_SENSOR_DISTANCE * unit_x),
                min(surf[1], surf[1] + ROBOT_MAX_SENSOR_DISTANCE * unit_y),
                max(surf[0], surf[0] + ROBOT_MAX_SENSOR_DISTANCE * unit_x),
                max(surf[1], surf[1] + ROBOT_MAX_SENSOR_DISTANCE * unit_y) )
        outside_bot = (surf[0] + unit_x * ROBOT_MAX_SENSOR_DISTANCE, surf[1] + unit_y * ROBOT_MAX_SENSOR_DISTANCE)
        obstacles_in_ROI = filter(lambda obs: Util.is_bbox_overlap(ROI, (obs.x, obs.y, obs.x + obs.width, obs.y + obs.height)), self._sm.obstacles)
        obstacle_bounding_lines = Util.all_bounding_lines_generator(obstacles_in_ROI)
        min_distance_to_wall_and_obs = min(Robot.distance_to_line_generators(surf, outside_bot, chain(SIMBOTMAP_BBOX, obstacle_bounding_lines)))
        
        if self._sm.robot_see_each_other:
            other_robots_in_ROI = filter(lambda r: r != self and Util.is_bbox_overlap(ROI, (r.x, r.y, r.x + r.width, r.y + r.height)), self._sm._robot_list)
            min_distance_to_other_robot = min(Robot.distance_to_robot_generators(surf, outside_bot, other_robots_in_ROI))
            return min(min_distance_to_wall_and_obs, min_distance_to_other_robot)
        else:
            return min_distance_to_wall_and_obs

    def _is_robot_inside_map(self, p: Util.Point2D=None) -> bool:
        if p is None:
            p = self.pos
        
        robot_radius = self.width / 2.0
        robot_center = (p[0] + robot_radius, p[1] + robot_radius)
        
        map_pos = self._sm.pos
        map_half_width = SIMBOTMAP_SIZE[0] / 2.0
        map_half_height = SIMBOTMAP_SIZE[1] / 2.0
        map_center = (map_pos[0] + map_half_width, map_pos[1] + map_half_height)

        dx = abs(robot_center[0] - map_center[0])
        dy = abs(robot_center[1] - map_center[1])
        
        if dx > (map_half_width - robot_radius) or dy > (map_half_height - robot_radius):
            return False
        return True

    def _is_robot_collide_obstacles(self, p: Util.Point2D, obstacles_included=None) -> bool:
        if obstacles_included is None:
            obstacles_included = self._sm.obstacles

        if p is None:
            p = self.pos

        robot_radius = self.width / 2.0
        robot_center = (p[0] + robot_radius, p[1] + robot_radius)

        # Check obstacles
        for obs in obstacles_included:
            obs_pos = obs.pos
            obs_width, obs_height = obs.size
            obs_center = (obs_pos[0] + obs_width / 2.0, obs_pos[1] + obs_height / 2.0)

            if Util.is_circle_rect_intersect(robot_center, robot_radius, obs_center, obs_width, obs_height):
                return True

        return False

    def _is_robot_collide_others(self, p: Util.Point2D):
        if p is None:
            p = self.pos
        
        robot_radius = self.width / 2.0
        robot_center = (p[0] + robot_radius, p[1] + robot_radius)

        for r in self._sm._robot_list:
            if r != self and Util.distance(r.center, robot_center) <= 2 * robot_radius:
                return True
        
        return False

    def _is_valid_position(self, next_position: Util.Point2D) -> bool:

        if not self._is_robot_inside_map(next_position):
            return False

        if self._is_robot_collide_obstacles(next_position):
            return False

        if self._sm.robot_see_each_other and self._is_robot_collide_others(next_position):
            return False
        
        return True

    def _get_overlap_objective(self) -> Objective:
        robot_center = self.center
        robot_radius = self.size[0] / 2.0
        for obj in self._sm.objectives:
            obj_width, obj_height = obj.size
            obj_center = (obj.pos[0] + obj_width / 2.0, obj.pos[1] + obj_height / 2.0)
            if Util.is_circle_rect_intersect(robot_center, robot_radius, obj_center, obj_width, obj_height):
                return obj
        return None
        
    def set_color(self, r: float, g: float, b: float, a: float=1) -> None:
        self._color_r = r
        self._color_g = g
        self._color_b = b
        self._color_a = a

    def distance(self) -> Sequence[float]:
        return tuple(self._distance(angle) for angle in ROBOT_DISTANCE_ANGLES)
    
    def smell(self, index: int = 0) -> float:
        if index >= 0 and index < len(self._sm.objectives):
            # Get angle
            obj = self._sm.objectives[index]
            dvx = self.center_x - obj.center_x
            dvy = self.center_y - obj.center_y
            rad = math.atan2(dvy, dvx)
            deg = ((180 - (math.degrees(rad) + self._direction)) % 360)
            if(deg <= 180):
                return deg
            else:
                return deg - 360
        return 0

    def smell_nearest(self) -> float:
        nearest_food = min(self._sm.objectives, key=lambda food: Util.distance(self.pos, food.pos))
        obj = nearest_food
        dvx = self.center_x - obj.center_x
        dvy = self.center_y - obj.center_y
        rad = math.atan2(dvy, dvx)
        deg = ((180 - (math.degrees(rad) + self._direction)) % 360)
        if(deg <= 180):
            return deg
        else:
            return deg - 360

    def turn(self, degree: float = 1) -> None:
        self._direction = (self._direction + degree) % 360
        self.stuck = False

    def move(self, step: int = 1) -> None:
        rad_angle = math.radians((-self._direction) % 360)
        if step >= 0:
            step = int(step)
        else:
            rad_angle = math.radians((180-self._direction) % 360)
            step = int(-step)
        dx = math.cos(rad_angle)
        dy = math.sin(rad_angle)

        self.stuck = False
        next_position = self.pos
        for distance in range(0, step, 1):
            next_possible_position = (next_position[0] + dx, next_position[1] + dy)
            # If can move
            if not self._is_valid_position(next_possible_position):
                if distance == 0:
                    self.stuck = True
                self.collision_count += 1
                break
            next_position = next_possible_position
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

class RobotWrapper(Widget):
    def get_robots(self) -> Sequence[Robot]:
        return [robot for robot in self.children if isinstance(robot, Robot)]