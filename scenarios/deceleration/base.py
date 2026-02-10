from core.scenario_manager import *
from core.trigger_condition import *

def make_deceleration_scenario(network,
                     ego_init_laneoffset,
                     ego_goal_laneoffset,
                     npc_init_laneoffset,
                     _speed,
                     acceleration=7,
                     body_style=BodyStyle.HATCHBACK):

    # ego specification
    ego = EgoVehicle(init_pose=Pose.from_lane_offset(ego_init_laneoffset, network),
                     goal_pose=Pose.from_lane_offset(ego_goal_laneoffset, network),
                     speed_limit=_speed)
    ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))

    # calculate the longitudinal distance to trigger the movement of the NPC so that 
    # after accelerating to _speed, longitudinal distance between ego and NPC is dx0=2.0*_speed
    dx0 = 2.0 * _speed
    speedup_time = _speed/acceleration
    speedup_dis = _speed**2 / 2 / acceleration
    # distance ego travels during the time NPC speeds up to _speed
    dis_ego = _speed * speedup_time
    dx = dx0 + dis_ego - speedup_dis

    # NPC specification
    npc1 = NPCVehicle("npc1", body_style,
                      init_pose=Pose.from_lane_offset(npc_init_laneoffset, network))
    npc1.add_action(FollowLane(target_speed=_speed,
                               acceleration=acceleration,
                               condition=longitudinal_distance_to_ego <= dx))
    npc1.add_action(SetTargetSpeed(target_speed=0,
                                   acceleration=-9.8,
                                   condition=actor_speed >= _speed))
    return Scenario(network, [ego, npc1])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  make_deceleration_scenario(scenario_manager.network,
                                    ego_init_laneoffset=LaneOffset('111', 0),
                                    ego_goal_laneoffset=LaneOffset('111', 210),
                                    npc_init_laneoffset=LaneOffset('111', 90),
                                    _speed=30 / 3.6
                                    )
    scenario_manager.run([scenario])