# Represent the pose (position + orientation) of an actor
# A pose can be constructed from 
# 1. LaneOffset: e.g., LaneOffset('111', 130)
# 2. Direct 3D point + yaw angle: e.g., position=(x,y,z), yaw=theta_deg
# 3. Relative to a reference LaneOffset:
#   3.1. left lane of LaneOffset('111', 130), forward 20m (note the curve lanes)
#   3.2. right lane of LaneOffset('111', 130), backward 10m (note the curve lanes)
#   3.4. same lane of LaneOffset('111', 130), forward 15m (note the curve lanes)
# 4. Relative to another Pose:
#   4.1. from Pose1, left 3m, forward 5m according to Pose1's heading
#   4.2. from Pose1, right 2m, backward 4m according to Pose1's heading

import numpy as np
import math
from map.network import LaneOffset
import utils

class Pose:
    """
    Represents the pose (position + orientation) of an actor.
    
    Attributes:
        position: np.ndarray of shape (3,), representing (x, y, z) coordinates
        orientation: list of 4 floats, representing quaternion (x, y, z, w)
        yaw: float, yaw angle in degrees
    """
    
    def __init__(self, position=None, orientation=None, yaw=None):
        """
        Flexible constructor that can handle various input formats.
        Subclasses or factory methods should be used for different initialization types.
        """
        if position is not None and orientation is not None:
            self.position = np.array(position, dtype=float)
            self.orientation = orientation
            self.yaw = yaw if yaw is not None else self._quaternion_to_yaw(orientation)
        else:
            raise ValueError("Both position and orientation must be provided")
    
    @staticmethod
    def _quaternion_to_yaw(quaternion):
        """
        Convert quaternion to yaw angle in degrees.
        Assumes rotation around Z-axis only (quaternion represents Z-axis rotation).
        
        :param quaternion: list of 4 floats [x, y, z, w]
        :return: yaw angle in degrees
        """
        qx, qy, qz, qw = quaternion
        yaw_rad = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        return math.degrees(yaw_rad)
    
    @classmethod
    def from_lane_offset(cls, lane_offset, network):
        """
        Create a Pose from a LaneOffset.
        
        :param lane_offset: LaneOffset object with lane_str and offset
        :param network: Network object for parsing the lane offset
        :return: Pose instance
        """
        _, _, position, quaternion = network.parse_lane_offset(lane_offset, extend_following_lane=True)
        # Convert quaternion (which is from yaw in radians) to yaw in degrees
        yaw_rad = math.atan2(2 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1]),
                            1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]))
        yaw_deg = math.degrees(yaw_rad)
        return cls(position=position, orientation=quaternion, yaw=yaw_deg)
    
    @classmethod
    def from_position_yaw(cls, position, yaw_deg):
        """
        Create a Pose from a 3D position and yaw angle in degrees.
        
        :param position: tuple or array of (x, y, z)
        :param yaw_deg: yaw angle in degrees
        :return: Pose instance
        """
        position = np.array(position, dtype=float)
        yaw_rad = math.radians(yaw_deg)
        quaternion = utils.quaternion_from_yaw(yaw_rad)
        return cls(position=position, orientation=quaternion, yaw=yaw_deg)
    
    @classmethod
    def from_left_lane(cls, reference_lane_offset, network, forward=0, backward=0, lane_width=3.5):
        """
        Create a Pose on the left adjacent lane of a reference LaneOffset.
        
        :param reference_lane_offset: LaneOffset object
        :param network: Network object
        :param forward: longitudinal distance forward
        :param backward: longitudinal distance backward
        :param lane_width: width of the lane (default: 3.5 m)
        :return: Pose instance
        """
        return cls.from_relative_to_lane(reference_lane_offset, network, 
                                         left=lane_width, forward=forward, backward=backward)
    
    @classmethod
    def from_right_lane(cls, reference_lane_offset, network, forward=0, backward=0, lane_width=3.5):
        """
        Create a Pose on the right adjacent lane of a reference LaneOffset.
        
        :param reference_lane_offset: LaneOffset object
        :param network: Network object
        :param forward: longitudinal distance forward
        :param backward: longitudinal distance backward
        :param lane_width: width of the lane (default: 3.5 m)
        :return: Pose instance
        """
        return cls.from_relative_to_lane(reference_lane_offset, network, 
                                         right=lane_width, forward=forward, backward=backward)

    @classmethod
    def from_relative_to_lane(cls, reference_lane_offset, network, left=0, right=0, forward=0, backward=0):
        """
        Create a Pose relative to a reference LaneOffset.
        
        The process:
        1. Get the reference position from the LaneOffset
        2. Calculate the lateral offset (left/right from the lane center)
        3. Find the lane that contains the laterally-offset point
        4. Apply the longitudinal offset (forward/backward) along the found lane
        
        :param reference_lane_offset: LaneOffset object
        :param network: Network object
        :param left: lateral distance to the left (mutually exclusive with right)
        :param right: lateral distance to the right (mutually exclusive with left)
        :param forward: longitudinal distance forward (mutually exclusive with backward)
        :param backward: longitudinal distance backward (mutually exclusive with forward)
        :return: Pose instance
        """
        # Get reference position and its lane
        _, ref_lane, ref_position, ref_quaternion = network.parse_lane_offset(reference_lane_offset)
        
        # Calculate yaw angle at reference position
        ref_yaw_rad = math.atan2(2 * (ref_quaternion[3] * ref_quaternion[2] + ref_quaternion[0] * ref_quaternion[1]),
                                1 - 2 * (ref_quaternion[1] * ref_quaternion[1] + ref_quaternion[2] * ref_quaternion[2]))
        
        # Calculate lateral distance (positive = left, negative = right)
        lateral_distance = left - right
        
        # Calculate the perpendicular direction (perpendicular to lane heading in 2D)
        # If heading is (cos(yaw), sin(yaw)), perpendicular left is (-sin(yaw), cos(yaw))
        perp_left_x = -math.sin(ref_yaw_rad)
        perp_left_y = math.cos(ref_yaw_rad)
        
        # Calculate the offset point from the reference position
        offset_point_2d = ref_position[:2] + lateral_distance * np.array([perp_left_x, perp_left_y])
        
        # Find the lane that contains this offset point
        target_lane = network.get_lane_from_point(offset_point_2d)
        if target_lane is None:
            raise ValueError(f"No lane found for lateral offset point {offset_point_2d}")
        
        # Find the offset value on the target lane
        # Calculate distance along the lane from start to the projection point
        longitudinal_offset_on_lane = 0
        projection_point_on_lane = None
        
        for i in range(len(target_lane.way_points) - 1):
            start = target_lane.way_points[i]
            end = target_lane.way_points[i + 1]
            segment = end - start
            segment_length = np.linalg.norm(segment)
            
            projection, is_inside_segment = utils.project_point_to_segment_2d(offset_point_2d, start[:2], end[:2])
            
            if is_inside_segment:
                projection_point_on_lane = projection
                longitudinal_offset_on_lane += np.linalg.norm(projection - start[:2])
                break
            else:
                longitudinal_offset_on_lane += segment_length
        
        if projection_point_on_lane is None:
            raise ValueError(f"Could not project offset point onto target lane")
        
        # Apply forward/backward offset
        longitudinal_offset = forward - backward
        final_lane_offset = LaneOffset(target_lane.id, longitudinal_offset_on_lane + longitudinal_offset)
        
        # Parse the final lane offset to get position and orientation
        return cls.from_lane_offset(final_lane_offset, network)
    
    @classmethod
    def from_relative_to_pose(cls, reference_pose, network=None, left=0, right=0, forward=0, backward=0):
        """
        Create a Pose relative to another Pose.
        
        The offsets are applied relative to the reference pose's heading direction.
        
        :param reference_pose: Pose object (reference)
        :param network: Network object (not strictly needed but kept for consistency)
        :param left: lateral distance to the left
        :param right: lateral distance to the right
        :param forward: longitudinal distance forward
        :param backward: longitudinal distance backward
        :return: Pose instance
        """
        # Get reference position and yaw
        ref_position = reference_pose.position.copy()
        ref_yaw_rad = math.radians(reference_pose.yaw)
        
        # Calculate the direction vector along the pose's heading
        forward_x = math.cos(ref_yaw_rad)
        forward_y = math.sin(ref_yaw_rad)
        
        # Calculate the perpendicular direction (left)
        left_x = -math.sin(ref_yaw_rad)
        left_y = math.cos(ref_yaw_rad)
        
        # Calculate the offset
        lateral_distance = left - right
        longitudinal_distance = forward - backward
        
        # Calculate new position
        new_position = ref_position.copy()
        new_position[0] += longitudinal_distance * forward_x + lateral_distance * left_x
        new_position[1] += longitudinal_distance * forward_y + lateral_distance * left_y
        
        # Orientation remains the same as reference
        return cls(position=new_position, orientation=reference_pose.orientation, yaw=reference_pose.yaw)
