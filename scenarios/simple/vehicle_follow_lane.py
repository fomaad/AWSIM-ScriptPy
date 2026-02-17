from core.scenario_manager import *
from core.trigger_condition import *

scenario_manager = ScenarioManager()
network = scenario_manager.network
npc1 = NPCVehicle("npc1", BodyStyle.SMALL_CAR, init_pose=Pose.from_lane_offset(LaneOffset('111'), network))
npc1.add_action(FollowLane(target_speed=30/3.6, acceleration=5))
npc1.add_action(ChangeLane(next_lane=network.parse_lane('112'), lateral_velocity=1.2,
                           condition=reach_point(LaneOffset('111', 20), network)))
scenario = Scenario(network, [npc1])
scenario_manager.run([scenario])