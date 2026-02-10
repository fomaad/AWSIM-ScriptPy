import json
from enum import Enum
from typing import Optional, Tuple
import numpy as np

import utils


class TurnDirection(Enum):
    UNDEFINED = 0
    STRAIGHT = 1
    LEFT = 2
    RIGHT = 3

# Represent a traffic lane
class TrafficLane:
    def __init__(self, id: str, turn_direction:TurnDirection, speed_limit:float,
                 width, way_points, next_lanes: Tuple[str], prev_lanes: Tuple[str]):
        self.id = id
        self.turn_direction = turn_direction
        self.speed_limit = speed_limit
        self.width = width

        self.way_points = way_points
        self.next_lanes = next_lanes
        self.prev_lanes = prev_lanes

        # self.stop_line = None   TODO: parse this info
        # self.right_of_way_lanes = []   TODO: parse this info

    @classmethod
    def from_dict(cls, dict_obj):
        way_points = [utils.dict_to_point_obj(p) for p in dict_obj['waypoints']]
        return cls(dict_obj['id'], dict_obj['turn_direction'], dict_obj['speed_limit'],
                   dict_obj['width'], way_points, dict_obj['next_lanes'], dict_obj['prev_lanes'])

    def is_intersection_lane(self):
        return self.turn_direction != TurnDirection.UNDEFINED
    
    def get_2D_waypoints(self):
        return [(x,y) for (x,y,_) in self.way_points]

    def has_id(self, _id):
        return self.id == _id or self.id == "TrafficLane." + _id
    
    def __str__(self):
        next_ids = [entry for entry in self.next_lanes]
        prev_ids = [entry for entry in self.prev_lanes]
        return f'Lane #{self.id}, next: {next_ids}, prev: {prev_ids}'

    def parse_wp_segment(self, point):
        """
        Get the segment on which the given point is located
        :param point:
        :return: the start index of the segment
        """
        for i in range(len(self.way_points) - 1):
            _, projection_inside_segment = utils.project_point_to_segment_2d(point[:2],
                                                                  self.way_points[i][:2],
                                                                  self.way_points[i + 1][:2])
            if projection_inside_segment:
                return i
        return -1

    def project_point2D_onto_lane(self, point2D):
        """
        :param point2D: np array
        :return: pair {projection, flag}, where flag is true iff the projection falls inside the lane.
        """
        min_dis = float("inf")
        result = None
        for i in range(len(self.way_points) - 1):
            projection, projection_inside_segment = utils.project_point_to_segment_2d(point2D,
                                                                                      self.way_points[i][:2],
                                                                                      self.way_points[i+1][:2])
            dis = np.linalg.norm(projection - point2D)
            if projection_inside_segment and dis < min_dis:
                min_dis = dis
                result = projection
        if result is not None:
            return result, True

        if np.linalg.norm(point2D - self.way_points[0][:2]) < np.linalg.norm(point2D - self.way_points[-1][:2]):
            return utils.project_point_to_segment_2d(point2D, self.way_points[0][:2], self.way_points[1][:2])
        return utils.project_point_to_segment_2d(point2D, self.way_points[-1][:2], self.way_points[-2][:2])