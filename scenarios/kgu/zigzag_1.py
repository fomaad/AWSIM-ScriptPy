# 2025/11/21
# modified from ziczac2.py 
# the width of weaving is larger

import copy

from core.scenario_manager import *
from core.trigger_condition import autonomous_mode_ready
from scenarios.utils import *

def make_scenario(network,
                  ego_init_laneoffset,
                  ego_goal_laneoffset,
                  npc_init_laneoffset,
                  start_laneoffset,
                  ego_speed,
                  npc_speed,
                  acceleration=0.5,
                  body_style=BodyStyle.SMALL_CAR):

    # ego specification
    ego = EgoVehicle(init_pose=Pose.from_lane_offset(ego_init_laneoffset, network),
                     goal_pose=Pose.from_lane_offset(ego_goal_laneoffset, network),
                     speed_limit=ego_speed)
    ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))

    # NPC specification
    _, _, npc_init_pos, _ = network.parse_lane_offset(npc_init_laneoffset)
    _id, source_lane, wp1, _ = network.parse_lane_offset(start_laneoffset)

    # ziczac waypoints
    waypoints = [npc_init_pos, wp1]

    direction = (source_lane.way_points[_id + 1] - wp1)[:2]
    direction_normalized = direction / np.linalg.norm(direction)

    long_shift = 8
    lat_shift = 2.7
    wp2 = point_forward(wp1[:2], direction_normalized, long_shift, lat_shift)
    waypoints.append(np.append(wp2, wp1[2]))

    right_point = wp2
    for i in range(1, 7):
        wp3 = right_point + direction_normalized * 2
#        wp4 = point_forward(wp3, direction_normalized, 6, -1.0)  
        wp4 = point_forward(wp3, direction_normalized, 6, -3.0)    # by KGU
        wp5 = wp4 + direction_normalized * 2    
#        wp6 = point_forward(wp5, direction_normalized, 6, 1.0)
        wp6 = point_forward(wp5, direction_normalized, 6, 3.0)   # by KGU
        for p in [wp3, wp4, wp5, wp6]:
            waypoints.append(np.append(p, wp1[2]))
        right_point = copy.deepcopy(wp6)

    npc1 = NPCVehicle("npc1", body_style,
                      init_pose=Pose.from_lane_offset(npc_init_laneoffset, network))
    npc1.add_action(FollowWaypoints(waypoints=[utils.array_to_dict_pos(p) for p in waypoints],
                                    target_speed=npc_speed,
                                    acceleration=acceleration,
                                    condition=autonomous_mode_ready()))

    return Scenario(network, [ego,npc1])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  make_scenario(scenario_manager.network,
                    ego_init_laneoffset=LaneOffset('112', 20),
                    ego_goal_laneoffset = LaneOffset('112', 150),
                    npc_init_laneoffset=LaneOffset('111', 35),
                    start_laneoffset=LaneOffset('111', 35),
                    ego_speed=30/3.6,
                    npc_speed=15/3.6)
    scenario_manager.run([scenario])
