from core.scenario_manager import *
from core.trigger_condition import *
from scenarios.utils import *

def make_swerve_scenario(network,
                         ego_init_laneoffset,
                         ego_goal_laneoffset,
                         npc_init_laneoffset,
                         _ego_speed,
                         _npc_speed,
                         swerve_vy,
                         dx0,
                         swerve_ny=1.8,
                         swerve_dis=2.0,
                         swerve_right=True,
                         acceleration=7,
                         body_style=BodyStyle.HATCHBACK):

    # ego specification
    ego = EgoVehicle(init_pose=Pose.from_lane_offset(ego_init_laneoffset, network),
                     goal_pose=Pose.from_lane_offset(ego_goal_laneoffset, network),
                     speed_limit=_ego_speed)
    ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))

    # function to dynamically calculate swerve waypoints based on the world state
    def cal_waypoints(actor, global_state):
        npc_kin = global_state['actor-kinematics']['vehicles'].get(actor.actor_id)
        npc_pos = np.array(npc_kin['pose']['position'])
        npc_front_center = actor.get_front_center(npc_pos, npc_kin['pose']['rotation'][2])

        angle = np.asin(swerve_vy / _npc_speed)
        x_shift = swerve_vy / np.tan(angle)
        direction = (npc_front_center - npc_pos)[:2]
        direction_normalized = direction / np.linalg.norm(direction)
        wp1 = point_forward(npc_front_center[:2],
                            direction_normalized,
                            x_shift,
                            swerve_ny if swerve_right else -swerve_ny)

        wp2 = wp1 + direction_normalized * swerve_dis
        wp3 = point_forward(wp2,
                            direction_normalized,
                            x_shift,
                            -swerve_ny if swerve_right else swerve_ny)

        wp4 = wp3 + direction_normalized * utils.extended_point_scale(_npc_speed)

        waypoints = [npc_front_center]
        for p in [wp1, wp2, wp3, wp4]:
            waypoints.append(np.append(p, npc_front_center[2]))

        return [utils.array_to_dict_pos(p) for p in waypoints]

    # npc specification
    speedup_time = _npc_speed / acceleration
    speedup_dis = 0.5 * _npc_speed**2 / acceleration + speedup_time * _ego_speed
    npc1 = NPCVehicle("npc1", body_style,
                      init_pose=Pose.from_lane_offset(npc_init_laneoffset, network))
    npc1.add_action(FollowLane(target_speed=_npc_speed,
                               acceleration=acceleration,
                               condition=longitudinal_distance_to_ego <= dx0 + speedup_dis + 1))
                               # condition=av_speed >= _ego_speed - 0.1))
    npc1.add_action(FollowWaypoints(waypoints_calculation_callback=cal_waypoints,
                                    condition=longitudinal_distance_to_ego <= dx0))

    return Scenario(network, [ego, npc1])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario = make_swerve_scenario(scenario_manager.network,
                                 ego_init_laneoffset=LaneOffset('355', 20),
                                 ego_goal_laneoffset=LaneOffset('214', 5),
                                 npc_init_laneoffset=LaneOffset('205', 65),
                                 _ego_speed=30/3.6,
                                 _npc_speed=10/3.6,
                                 swerve_vy=1.2,
                                 dx0=26
                                )
    scenario_manager.run([scenario])