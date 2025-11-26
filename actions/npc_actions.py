from core.action import Action
import std_msgs, rclpy
import json, time
import utils
from core.actor import NPCVehicle
from core.client_ros_node import *

class SpawnNPCVehicle(Action):
    def __init__(self, position=None, orientation=None,
                 pose_callback=None, condition=None):
        """
        Either {position, orientation} or pose_callback must be defined.
        :param position: np array
        :param orientation: np array
        :param pose_callback: function that takes (actor, global_state) and returns (position, orientation)
        :return:
        """
        super().__init__(condition=condition, one_shot=True)
        self.position = position
        self.orientation = orientation
        self.pose_callback = pose_callback

    def _do(self, actor:NPCVehicle, client_node, global_state):
        pos = self.position
        orient = self.orientation
        if self.pose_callback is not None:
            pos, orient = self.pose_callback(actor, global_state)
        my_dict = {
            "name": actor.actor_id,
            "body_style": actor.body_style.value,
            "position": utils.array_to_dict_pos(pos),
            "orientation": utils.array_to_dict_orient(orient)
        }

        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        client_node.dynamic_npc_spawning_publisher.publish(msg)

        client_node.get_logger().info(f"Spawned NPC vehicle {actor.actor_id}")

class FollowLane(Action):
    def __init__(self, lane=None, condition=None, target_speed=None, acceleration=None):
        super().__init__(condition=condition, one_shot=True)

        # if lane is None, following the lane on which the actor currently located.
        # Once finish the lane, randomly follow one of the next lanes
        # If specified, it must be a string
        self.lane = lane

        # if target_speed is None, it follows the speed limit of the current lane
        # if acceleration is None, the default values will be used
        self.target_speed = target_speed
        self.acceleration = acceleration

    def _do(self, actor:NPCVehicle, client_node, global_state):
        is_speed_defined = self.target_speed is not None
        is_acceleration_defined = self.acceleration is not None
        my_dict = {
            "target": actor.actor_id,
            "lane": "" if self.lane is None else self.lane,
            "speed": self.target_speed if is_speed_defined else 0,
            "acceleration": math.fabs(self.acceleration) if is_acceleration_defined else 0,
            "is_speed_defined": is_speed_defined,
            "is_acceleration_defined": is_acceleration_defined,
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        client_node.follow_lane_publisher.publish(msg)
        client_node.get_logger().info(f"Sent follow lane command to {actor.actor_id} successfully.")

class FollowWaypoints(Action):
    def __init__(self, waypoints=None, waypoints_calculation_callback=None,
                 condition=None, target_speed=None, acceleration=None):
        """
        Either waypoints or waypoints_calculation_callback must be defined.
        :param waypoints:
        :param waypoints_calculation_callback:
        :param condition:
        :param target_speed:
        :param acceleration:
        """
        if waypoints is None and waypoints_calculation_callback is None:
            raise ValueError("waypoints or waypoints_calculation_callback must be defined")
        super().__init__(condition=condition, one_shot=True)
        self.waypoints = waypoints
        self.waypoints_calculation_callback = waypoints_calculation_callback

        # if target_speed is None, it follows the speed limit of the current lane
        # if acceleration is None, the default values will be used
        self.target_speed = target_speed
        self.acceleration = acceleration

    def _do(self, actor:NPCVehicle, client_node, global_state):
        waypoints = self.waypoints
        if waypoints is None:
            waypoints = self.waypoints_calculation_callback(actor, global_state)
        is_speed_defined = self.target_speed is not None
        is_acceleration_defined = self.acceleration is not None

        my_dict = {
            "target": actor.actor_id,
            "waypoints": waypoints,
            "speed": self.target_speed if is_speed_defined else 0,
            "acceleration": math.fabs(self.acceleration) if is_acceleration_defined else 0,
            "is_speed_defined": is_speed_defined,
            "is_acceleration_defined": is_acceleration_defined,
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        client_node.follow_waypoints_publisher.publish(msg)
        client_node.get_logger().info(f"Sent follow waypoints command to {actor.actor_id} successfully.")

class SetTargetSpeed(Action):
    def __init__(self, target_speed, condition=None, acceleration=None):
        super().__init__(condition=condition, one_shot=True)

        # if acceleration is None, the default values will be used
        self.target_speed = target_speed
        self.acceleration = acceleration

    def _do(self, actor:NPCVehicle, client_node, global_state):
        is_acceleration_defined = self.acceleration is not None
        my_dict = {
            "target": actor.actor_id,
            "speed": self.target_speed,
            "acceleration": math.fabs(self.acceleration) if is_acceleration_defined else 0,
            "is_acceleration_defined": is_acceleration_defined,
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        client_node.set_target_speed_publisher.publish(msg)
        client_node.get_logger().info(f"Sent set target speed to {actor.actor_id} successfully.")

class ChangeLane(Action):
    def __init__(self, next_lane, lateral_velocity=1.0,
                 condition=None):
        super().__init__(condition=condition, one_shot=True)
        self.next_lane = next_lane
        self.lateral_velocity = lateral_velocity

    def _do(self, actor:NPCVehicle, client_node, global_state):
        kinematic = global_state['actor-kinematics']['vehicles'].get(actor.actor_id)
        if not kinematic:
            client_node.get_logger().info(f'[ERROR] NPC {actor.actor_id} not found in the world state')
            return
        current_pos = np.array(kinematic['pose']['position'])
        current_speed = np.linalg.norm(np.array(kinematic['twist']['linear']))
        front_center_point = actor.get_front_center(current_pos, kinematic['pose']['rotation'][2])

        projection = None
        next_lane_wp_id = -1
        for i in range(len(self.next_lane.way_points) - 1):
            proj, projection_inside_segment = utils.project_point_to_segment_2d(
                front_center_point[:2],
                self.next_lane.way_points[i][:2],
                self.next_lane.way_points[i + 1][:2])
            if projection_inside_segment:
                projection = proj
                next_lane_wp_id = i

        if projection is None:
            client_node.get_logger().error(f'[ERROR] Invalid next lane for the lane change')
            return

        # calculate waypoints
        lateral_dis = np.linalg.norm(front_center_point[:2] - projection)
        if current_speed < self.lateral_velocity:
            client_node.get_logger().error(f'[ERROR] The lateral velocity ({self.lateral_velocity}) '
                                           f'is greater than the current speed ({current_speed}). Ignore lane change.')
            return

        long_speed = np.sqrt(current_speed ** 2 - self.lateral_velocity ** 2)
        long_dis = lateral_dis / self.lateral_velocity * long_speed
        direction = projection - self.next_lane.way_points[next_lane_wp_id][:2]
        direction = direction / np.linalg.norm(direction)
        wp0 = projection + direction * long_dis
        waypoints = [front_center_point, np.append(wp0, front_center_point[2])]

        # add one more waypoint for smooth steering back
        wp1 = wp0 + direction * utils.extended_point_scale(current_speed)
        waypoints.append(np.append(wp1, front_center_point[2]))

        next_wp_id = next_lane_wp_id + 1
        while next_wp_id < len(self.next_lane.way_points):
            next_wp = self.next_lane.way_points[next_wp_id][:2]
            if np.dot(direction, next_wp - wp1) > 0:
                break
            next_wp_id += 1

        if next_wp_id < len(self.next_lane.way_points):
            waypoints += self.next_lane.way_points[next_wp_id:]

        my_dict = {
            "target": actor.actor_id,
            "waypoints": [utils.array_to_dict_pos(p) for p in waypoints]
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        client_node.follow_waypoints_publisher.publish(msg)
        client_node.get_logger().info(f"Sent lane change command to {actor.actor_id}.")
