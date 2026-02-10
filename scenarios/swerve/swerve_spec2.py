from core.scenario_manager import *
from core.trigger_condition import *

def make_swerve_scenario(network,
                         ego_init_laneoffset,
                         ego_goal_laneoffset,
                         npc_init_laneoffset,
                         swerve_start_laneoffset,
                         ego_speed,
                         npc_speed,
                         swerve_vy,
                         dx0,
                         swerve_ny=1.8,
                         swerve_dis=2.0,
                         swerve_right=True,
                         acceleration=8,
                         body_style=BodyStyle.HATCHBACK,
                         delay_time=0.03):

    # ego specification
    ego = EgoVehicle(init_pose=Pose.from_lane_offset(ego_init_laneoffset, network),
                     goal_pose=Pose.from_lane_offset(ego_goal_laneoffset, network),
                     speed_limit=ego_speed)
    ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))

    _, _, npc_init_pos, _ = network.parse_lane_offset(npc_init_laneoffset)
    _id, source_lane, wp1, _ = network.parse_lane_offset(swerve_start_laneoffset)

    npc_root_to_frontcenter = NPCVehicle.get_size(body_style)[0]/2 + NPCVehicle.get_center_offset(body_style)[0]

    # swerve specification
    # 1st step: compute the waypoints
    waypoints = [npc_init_pos, wp1]
    angle = np.asin(swerve_vy / npc_speed)
    diagonal = npc_speed * swerve_ny / swerve_vy  # ny / sin(angle)
    direction = (source_lane.way_points[_id + 1] - wp1)[:2]
    direction_normalized = direction / np.linalg.norm(direction)
    wp1_2D = wp1[:2]
    wp2 = utils.rotate_point(wp1_2D + direction_normalized * diagonal,
                             wp1_2D,
                             angle if not swerve_right else -angle)

    wp3 = wp2 + direction_normalized * swerve_dis
    wp4 = wp1_2D + direction_normalized * (swerve_dis + 2 * np.cos(angle) * diagonal)

    waypoints.append(np.append(wp2, wp1[2]))
    waypoints.append(np.append(wp3, wp1[2]))
    waypoints.append(np.append(wp4, wp1[2]))

    # 2nd step: calculate distance to trigger npc movement
    speedup_time = npc_speed / acceleration
    speedup_dis = npc_speed ** 2 / 2 / acceleration
    npc_travel_dis = np.linalg.norm(wp1 - npc_init_pos) - npc_root_to_frontcenter
    remaining_dis_to_wp1 = npc_travel_dis - speedup_dis
    time_to_wp1 = speedup_time + remaining_dis_to_wp1 / npc_speed + delay_time
    ego_travel_dis = ego_speed * time_to_wp1
    dis_threshold = dx0 + ego_travel_dis + npc_travel_dis

    # npc and follow waypoints (swerve waypoints) specification
    npc1 = NPCVehicle("npc1", body_style,
                      init_pose=Pose.from_lane_offset(npc_init_laneoffset, network))
    npc1.add_action(FollowWaypoints(waypoints=[utils.array_to_dict_pos(p) for p in waypoints],
                                    target_speed=npc_speed,
                                    acceleration=acceleration,
                                    condition=longitudinal_distance_to_ego <= dis_threshold))

    # to show non-conservative
    # npc1.add_action(FollowLane(target_speed=npc_speed,
    #                         acceleration=acceleration,
    #                         condition=longitudinal_distance_to_ego <= dis_threshold))

    return Scenario(network, [ego, npc1])
