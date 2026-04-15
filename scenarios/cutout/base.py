from core.scenario_manager import *
from core.trigger_condition import *

def make_cutout_scenario(network,
                     ego_init_laneoffset,
                     ego_goal_laneoffset,
                     cutout_npc_init_laneoffset,
                     cutout_next_lane,
                     _speed,
                     vy,
                     dx_f,
                     acceleration=7,
                     body_style=BodyStyle.HATCHBACK):

    _, _, init_pos, init_orient = network.parse_lane_offset(ego_init_laneoffset)
    _, _, goal_pos, goal_orient = network.parse_lane_offset(ego_goal_laneoffset)

    # ego specification
    ego = EgoVehicle(init_pose=Pose.from_lane_offset(ego_init_laneoffset, network),
                     goal_pose=Pose.from_lane_offset(ego_goal_laneoffset, network),
                     speed_limit=_speed)
    ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))

    # calculate the longitudinal distance to trigger the movement of the cut-out NPC so that 
    # after accelerating to _speed, longitudinal distance between it and ego is dx0=2.0*_speed
    dx0 = 2.0 * _speed
    speedup_time = _speed/acceleration
    speedup_dis = _speed**2 / 2 / acceleration
    # distance ego travels during the time NPC speeds up to _speed
    dis_ego = _speed * speedup_time
    dx = dx0 + dis_ego - speedup_dis

    next_lane = network.parse_lane(cutout_next_lane)

    # cut-out NPC specification
    npc1 = NPCVehicle("npc1", BodyStyle.SMALL_CAR, 
                      init_pose=Pose.from_lane_offset(cutout_npc_init_laneoffset, network))
    npc1.add_action(FollowLane(target_speed=_speed,
                               acceleration=acceleration,
                               condition=longitudinal_distance_to_ego <= dx))
    npc1.add_action(ChangeLane(next_lane=next_lane,
                                lateral_velocity=vy,
                                condition=actor_speed >= _speed))
    
    # stopped NPC specification (challenging vehicle)
    npc1_root_to_frontcenter = npc1.size[0]/2 + npc1.center[0]
    npc2_root_to_rearcenter = NPCVehicle.get_size(body_style)[0]/2 - NPCVehicle.get_center_offset(body_style)[0]
    npc2_offset = cutout_npc_init_laneoffset.offset + dx_f + npc1_root_to_frontcenter + npc2_root_to_rearcenter
    npc2 = NPCVehicle("npc2", body_style,
                      init_pose=Pose.from_lane_offset(LaneOffset(cutout_npc_init_laneoffset.lane_str, npc2_offset), network))
    return Scenario(network, [ego, npc1, npc2])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  make_cutout_scenario(scenario_manager.network,
                                    ego_init_laneoffset=LaneOffset('111', 0),
                                    ego_goal_laneoffset=LaneOffset('111', 210),
                                    cutout_npc_init_laneoffset=LaneOffset('111', 80),
                                    _speed=30 / 3.6,
                                    cutout_next_lane='112',
                                    vy=1.5,
                                    dx_f=12.0
                                    )
    scenario_manager.run([scenario])