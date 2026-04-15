from core.scenario_manager import *
from core.trigger_condition import *

"""
A scenario with a pedestrian
"""

scenario_manager = ScenarioManager()
network = scenario_manager.network
npc_init_pose = Pose.from_lane_offset(LaneOffset('111'), network)
npc1 = NPCVehicle("npc1", BodyStyle.SMALL_CAR, init_pose=npc_init_pose)
npc1.add_action(FollowLane(target_speed=30/3.6, acceleration=5))

pedes_init_pose = Pose.from_relative_to_pose(npc_init_pose, network, left=6, forward=4)
pedes_wp1 = Pose.from_relative_to_pose(pedes_init_pose, network, forward=8)
pedes_wp2 = Pose.from_relative_to_pose(pedes_wp1, network, right=8, forward=3)
pedes1 = NPCPedestrian("ped1", PedestrianStyle.ELEGANT, init_pose=pedes_init_pose)
pedes1.add_action(Walking(waypoints=[pedes_init_pose, pedes_wp1, pedes_wp2], target_speed=1))
scenario = Scenario(network, [npc1, pedes1])
scenario_manager.run([scenario])