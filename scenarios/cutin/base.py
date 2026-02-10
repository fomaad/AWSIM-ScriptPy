from core.scenario_manager import *
from core.trigger_condition import *

def make_cutin_scenario(network,
                     ego_init_laneoffset,
                     ego_goal_laneoffset,
                     npc_init_laneoffset,
                     cutin_next_lane,
                     _ego_speed,
                     _npc_speed,
                     _cutin_vy,
                     dx0,
                     acceleration=7,
                     body_style=BodyStyle.HATCHBACK):

    # ego specification
    ego = EgoVehicle(init_pose=Pose.from_lane_offset(ego_init_laneoffset, network),
                     goal_pose=Pose.from_lane_offset(ego_goal_laneoffset, network),
                     speed_limit=_ego_speed)
    ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))

    # NPC specification
    npc1 = NPCVehicle("npc1", body_style, 
                      init_pose=Pose.from_lane_offset(npc_init_laneoffset, network))

    next_lane = network.parse_lane(cutin_next_lane)
    npc1.add_action(FollowLane(target_speed=_npc_speed,
                               acceleration=acceleration,
                               condition=av_speed >= _ego_speed-0.2))
    npc1.add_action(ChangeLane(next_lane=next_lane,
                               lateral_velocity=_cutin_vy,
                               condition=longitudinal_distance_to_ego <= dx0))
    return Scenario(network, [ego, npc1])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  make_cutin_scenario(scenario_manager.network,
                                    ego_init_laneoffset=LaneOffset('111', 0),
                                    ego_goal_laneoffset=LaneOffset('111', 130),
                                    npc_init_laneoffset=LaneOffset('112', 88),
                                    cutin_next_lane='111',
                                    _ego_speed=30 / 3.6,
                                    _npc_speed=10 / 3.6,
                                    _cutin_vy=1.2,
                                    dx0=12
                                    )
    scenario_manager.run([scenario])