from core.client_ros_node import AdsInternalStatus
from map.network import LaneOffset
import utils
import numpy as np

class Measurement:
    """
    Wraps a numeric-measurement function and provides comparison operators
    that return boolean condition callables (actor, global_state) -> bool.
    Example usage:
        # returns a condition function
        cond = longitudinal_distance_to_ego < 10
    """
    def __init__(self, func):
        self.func = func

    def value(self, actor, global_state):
        """
        Return numeric measurement or None if not available.
        """
        try:
            return self.func(actor, global_state)
        except Exception:
            return None

    def _make_comparison(self, op, other, swapped=False):
        # op: a binary function taking (a, b) and returning bool
        def _cond(actor, global_state):
            val = self.value(actor, global_state)
            if val is None:
                return False
            try:
                if swapped:
                    return op(other, val)
                return op(val, other)
            except Exception:
                return False

        return _cond

    # rich comparisons
    def __lt__(self, other):
        return self._make_comparison(lambda a, b: a < b, other)

    def __le__(self, other):
        return self._make_comparison(lambda a, b: a <= b, other)

    def __gt__(self, other):
        return self._make_comparison(lambda a, b: a > b, other)

    def __ge__(self, other):
        return self._make_comparison(lambda a, b: a >= b, other)

    # reflected comparisons to allow writing e.g. 10 > measurement
    def __rlt__(self, other):
        return self._make_comparison(lambda a, b: a < b, other, swapped=True)

    def __rle__(self, other):
        return self._make_comparison(lambda a, b: a <= b, other, swapped=True)

    def __rgt__(self, other):
        return self._make_comparison(lambda a, b: a > b, other, swapped=True)

    def __rge__(self, other):
        return self._make_comparison(lambda a, b: a >= b, other, swapped=True)

def _longitudinal_distance_to_ego_value(actor, global_state):
    if not global_state['actor-kinematics'] or not global_state['actor-sizes']:
        return None
    ego = global_state['actor-kinematics']['ego']
    ego_pos = np.array(ego['pose']['position'])
    ego_euler_angles = np.array(ego['pose']['rotation'])/180*np.pi

    npc = global_state['actor-kinematics']['vehicles'].get(actor.actor_id)
    if not npc:
        return None
    npc_pos = np.array(npc['pose']['position'])

    ego_heading = utils.normalize_angle_0_to_2pi(ego_euler_angles[2])
    npc_heading = utils.normalize_angle_0_to_2pi(npc['pose']['rotation'][2]/180*np.pi)

    dis = utils.longitudinal_distance(ego_pos, npc_pos, ego_euler_angles)
    ego_root_to_front = global_state["actor-sizes"]["ego"]["size"][0]/2 + \
                        global_state["actor-sizes"]["ego"]["center"][0]
    if abs(ego_heading - npc_heading) < 0.5:
        # same direction
        npc_root_to_back = global_state["actor-sizes"][actor.actor_id]["size"][0]/2 - \
                        global_state["actor-sizes"][actor.actor_id]["center"][0]
        return dis - ego_root_to_front - npc_root_to_back
    else:
        # opposite direction
        npc_root_to_front = global_state["actor-sizes"][actor.actor_id]["size"][0] / 2 + \
                            global_state["actor-sizes"][actor.actor_id]["center"][0]
        return dis - ego_root_to_front - npc_root_to_front

def _distance_to_ego_value(actor, global_state):
    if not global_state['actor-kinematics'] or not global_state['actor-sizes']:
        return None
    ego = global_state['actor-kinematics']['ego']
    ego_pos = np.array(ego['pose']['position'])

    npc = global_state['actor-kinematics']['vehicles'].get(actor.actor_id)
    if not npc:
        return None

    npc_pos = np.array(npc['pose']['position'])
    dis = np.linalg.norm(ego_pos - npc_pos)
    return dis

def _speed_value(actor, global_state):
    if not global_state['actor-kinematics']:
        return None

    npc = global_state['actor-kinematics']['vehicles'].get(actor.actor_id)
    if not npc:
        return None
    vel = np.array(npc['twist']['linear'])
    return np.linalg.norm(vel)

def _ego_speed_value(actor, global_state):
    if not global_state['actor-kinematics']:
        return None

    ego = global_state['actor-kinematics']['ego']
    vel = np.array(ego['twist']['linear'])
    return np.linalg.norm(vel)

""" Function-based conditions"""
def autonomous_mode_ready():
    def _cond(actor, global_state):
        return global_state['ads_internal_status'] >= AdsInternalStatus.AUTONOMOUS_MODE_READY.value
    return _cond

def end_lane(lane, network):
    lane_obj = network.parse_lane(lane)
    last_wp = lane_obj.way_points[-1]
    def _cond(actor, global_state):
        if not global_state['actor-kinematics']:
            return False
        npc = global_state['actor-kinematics']['vehicles'].get(actor.actor_id)
        if not npc:
            # print(f'[ERROR] NPC {actor.actor_id} not found')
            return False
        pos = np.array(npc['pose']['position'])
        front_center_point = actor.get_front_center(pos, npc['pose']['rotation'][2])
        return np.linalg.norm(front_center_point[:2] - last_wp[:2]) < 0.1
    return _cond

def reach_point(point, network):
    if isinstance(point, LaneOffset):
        _, _, nppoint, _ = network.parse_lane_offset(point)
    elif isinstance(point, np.ndarray):
        nppoint = point
    elif isinstance(point, list):
        nppoint = np.array(point)
    else:
        raise TypeError('Unsupported point type. Use LaneOffset or np.ndarray')

    def _cond(actor, global_state):
        if not global_state['actor-kinematics']:
            return False

        npc = global_state['actor-kinematics']['vehicles'].get(actor.actor_id)
        if not npc:
            # print(f'[ERROR] NPC {actor.actor_id} not found')
            return False
        pos = np.array(npc['pose']['position'])
        front_center_point = actor.get_front_center(pos, npc['pose']['rotation'][2])
        forward = front_center_point[:2] - pos[:2]
        temp = front_center_point[:2] - nppoint[:2]
        # we set a big-enough threshold distance threshold of 3m to account for the case
        # when the speed is large and sleeping time between two cycles is significant
        return np.linalg.norm(temp) < 3.0 and np.dot(forward, temp) >= 0
    return _cond

def heading_wrt_lane_gt(actor, network, threshold):
    def _cond(self_actor, global_state):
        npc = global_state['actor-kinematics']['vehicles'].get(actor.actor_id)
        if not npc:
            # print(f'[ERROR] NPC {actor.actor_id} not found')
            return False
        
        npc_heading = npc['pose']['rotation'][2]

        ego = global_state['actor-kinematics']['ego']
        ego_pos = np.array(ego['pose']['position'])
        lane, _, wp_id = network.get_lane_from_point(ego_pos)
        if not lane:
            print(f'Ego is not on any lane')
            return False

        lane_direction_vec = lane.way_points[wp_id + 1][:2] - lane.way_points[wp_id][:2]
        lane_heading = np.arctan2(lane_direction_vec[1], lane_direction_vec[0]) * 180 / np.pi
        # print(f"NPC {actor.actor_id} heading: {npc_heading}. Lane heading: {lane_heading}.")
        heading_diff = lane_heading - npc_heading
        # normalize the heading difference to be within [-180, 180]
        heading_diff = np.abs((heading_diff + 180) % 360 - 180)
        return heading_diff > threshold
    return _cond

distance_to_ego = Measurement(_distance_to_ego_value)
longitudinal_distance_to_ego = Measurement(_longitudinal_distance_to_ego_value)
actor_speed = Measurement(_speed_value)
av_speed = Measurement(_ego_speed_value)
