from typing import List
from map.traffic_lane import *
import numpy as np
import math

class LaneOffset:
    def __init__(self, lane_str, offset: float=0.0):
        self.lane_str = lane_str
        self.offset = offset

class Network:
    coordinate_type:bool = False
    traffic_lanes: List[TrafficLane]
    def __init__(self, json_str):
        obj = json.loads(json_str)
        self.coordinate_type = obj['coordinate_type']
        self.traffic_lanes = [TrafficLane.from_dict(lane) for lane in obj['traffic_lanes']]

    def parse_lane_offset(self, lane_offset, extend_following_lane: bool=False):
        """
        Derive the 3D point from the given lane offset
        :param lane_offset:
        :param extend_following_lane: Whether to extend the current lane if the offset exceeds the lane length
        :return: {id, point}, where id is the waypoint index of the start of
        the waypoint segment on which the point is located
        """
        lane = next((l for l in self.traffic_lanes if l.id == lane_offset.lane_str), None)
        if not lane:
            full_name = "TrafficLane." + lane_offset.lane_str
            lane = next((l for l in self.traffic_lanes if l.id == full_name), None)
            if not lane:
                raise Exception(f"Lane {lane_offset.lane_str} not found")

        remaining_dis = lane_offset.offset
        current_lane = lane

        while True:
            for i in range(len(current_lane.way_points) - 1):
                start = current_lane.way_points[i]
                end = current_lane.way_points[i + 1]
                segment = end - start
                if remaining_dis - np.linalg.norm(segment) < 0:
                    point = start + remaining_dis/np.linalg.norm(segment)*segment
                    yaw_angle = math.atan2(end[1] - point[1], end[0] - point[0])
                    return i, current_lane, point, utils.quaternion_from_yaw(yaw_angle)

                remaining_dis = remaining_dis - np.linalg.norm(segment)

            if extend_following_lane and len(current_lane.next_lanes) > 0:
                next_lane = self.parse_lane(current_lane.next_lanes[0])
                current_lane = next_lane
            else:
                raise Exception(f"Offset {lane_offset.offset} exceeds length of lane {lane_offset.lane_str}")

    def parse_lane(self, lane):
        """
        :param lane: can be string in full name or shortened form
        :return:
        """
        if isinstance(lane, TrafficLane):
            return lane
        if isinstance(lane, str):
            return next((l for l in self.traffic_lanes if l.has_id(lane)), None)
        raise Exception(f"Cannot interpret lane {lane}.")
    
    def get_lane_from_point(self, point, distance_threshold=0.5):
        """
        Get the lane on which the given point is located
        """
        for lane in self.traffic_lanes:
            proj_point, is_inside = lane.project_point2D_onto_lane(point[:2])
            if is_inside and np.linalg.norm(proj_point - point[:2]) <= distance_threshold:
                if len(point) >= 3:
                    # If the point has a z-coordinate, we need to check if it's on the lane's elevation
                    wp_id = lane.parse_wp_segment(proj_point)
                    lane_elevation = lane.way_points[wp_id][2] if wp_id >= 0 else None
                    if lane_elevation is not None and abs(lane_elevation - point[2]) < 4.0:
                        return lane, proj_point, wp_id
                else:
                    return lane, proj_point, None
        return None, None, None