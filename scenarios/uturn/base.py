from core.scenario_manager import *
from core.trigger_condition import *

def  make_uturn_scenario(network,
                         ego_init_laneoffset,
                         ego_goal_laneoffset,
                         npc_init_laneoffset,
                         uturn_next_lane,
                         _ego_speed,
                         _npc_speed,
                         dx0,
                         acceleration=7,
                         body_style=BodyStyle.TAXI):

    # ego specification
    ego = EgoVehicle(init_pose=Pose.from_lane_offset(ego_init_laneoffset, network),
                    goal_pose=Pose.from_lane_offset(ego_goal_laneoffset, network),
                    speed_limit=_ego_speed)
    ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))

    # function to dynamically calculate uturn waypoints based on the world state
    def cal_waypoints(actor, global_state):
        npc_kin = global_state['actor-kinematics']['vehicles'].get(actor.actor_id)
        npc_pos = np.array(npc_kin['pose']['position'])
        npc_front_center = actor.get_front_center(npc_pos, npc_kin['pose']['rotation'][2])

        next_lane = network.parse_lane(uturn_next_lane)
        proj,_ = next_lane.project_point2D_onto_lane(npc_pos[:2])
        turning_center = (proj + npc_pos[:2])/2
        turning_side = utils.get_point_side(npc_pos[:2], npc_front_center[:2], turning_center)

        waypoints = [npc_front_center]
        for i in range(1, 4):
            wp = utils.rotate_point(npc_front_center[:2], turning_center, turning_side*i*np.pi/3)
            waypoints.append(np.append(wp, npc_front_center[2]))

        direction = (npc_front_center - npc_pos)[:2]
        direction_normalized = direction / np.linalg.norm(direction)
        extended_wp = waypoints[-1][:2] - direction_normalized * utils.extended_point_scale(_npc_speed)
        waypoints.append(np.append(extended_wp, npc_front_center[2]))
        return [utils.array_to_dict_pos(p) for p in waypoints]

    # npc specification
    npc1 = NPCVehicle("npc1", body_style,
                      init_pose=Pose.from_lane_offset(npc_init_laneoffset, network))
    npc1.add_action(FollowLane(target_speed=_npc_speed,
                               acceleration=acceleration,
                               condition=av_speed >= _ego_speed - 0.2))
    npc1.add_action(FollowWaypoints(waypoints_calculation_callback=cal_waypoints,
                                    condition=longitudinal_distance_to_ego <= dx0))

    return Scenario(network, [ego, npc1])