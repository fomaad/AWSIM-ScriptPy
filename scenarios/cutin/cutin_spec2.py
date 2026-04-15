"""
Another specification for the cutin scenarios
"""
from core.scenario_manager import *
from core.trigger_condition import *

def make_cutin_scenario(node, network,
                     ego_init_laneoffset,
                     ego_goal_laneoffset,
                     npc_init_laneoffset,
                     cutin_start_laneoffset,
                     cutin_next_lane,
                     ego_speed,
                     npc_speed,
                     cutin_vy,
                     dx0,
                     acceleration=5,
                     body_style=BodyStyle.HATCHBACK):

    # ego specification
    ego = EgoVehicle(init_pose=Pose.from_lane_offset(ego_init_laneoffset, network),
                     goal_pose=Pose.from_lane_offset(ego_goal_laneoffset, network),
                     speed_limit=ego_speed)
    ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))

    _, _, npc_init_pos, _ = network.parse_lane_offset(npc_init_laneoffset)
    _id, source_lane, wp1, _ = network.parse_lane_offset(cutin_start_laneoffset)

    # cutin specification
    next_lane = network.parse_lane(cutin_next_lane)
    proj, _ = next_lane.project_point2D_onto_lane(wp1[:2])
    lateral_dis = np.linalg.norm(proj - wp1[:2])

    # compute the waypoints
    waypoints = [npc_init_pos, wp1]
    angle = np.asin(cutin_vy / npc_speed)
    diagonal = npc_speed * lateral_dis / cutin_vy
    direction = (source_lane.way_points[_id + 1] - wp1)[:2]
    direction_normalized = direction / np.linalg.norm(direction)

    turning_side = utils.get_point_side(npc_init_pos[:2], wp1[:2], proj)
    wp2 = utils.rotate_point(wp1[:2] + direction_normalized * diagonal,
                             wp1[:2],
                             turning_side * angle)
    straight_direction = (wp1 - npc_init_pos)[:2]
    wp3 = wp2 + (0.4*npc_speed+5) * straight_direction/np.linalg.norm(straight_direction)
    waypoints.append(np.append(wp2, wp1[2]))
    waypoints.append(np.append(wp3, wp1[2]))

    # npc and follow waypoints (cutin waypoints) specification
    npc1 = NPCVehicle("npc1", body_style,
                      init_pose=Pose.from_lane_offset(npc_init_laneoffset, network))
    npc1.add_action(FollowWaypoints(waypoints=[utils.array_to_dict_pos(p) for p in waypoints],
                                    target_speed=npc_speed,
                                    acceleration=acceleration,
                                    condition=distance_to_ego <= dx0))

    return Scenario(network, [ego, npc1])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  make_cutin_scenario(scenario_manager.client_node, scenario_manager.network,
                                    ego_init_laneoffset=LaneOffset('111', 0),
                                    ego_goal_laneoffset=LaneOffset('111', 130),
                                    npc_init_laneoffset=LaneOffset('112', 88),
                                    cutin_start_laneoffset=LaneOffset('112', 95),
                                    cutin_next_lane='111',
                                    ego_speed=30 / 3.6,
                                    npc_speed=20 / 3.6,
                                    cutin_vy=1.2,
                                    dx0=20,
                                    )
    scenario_manager.run([scenario])