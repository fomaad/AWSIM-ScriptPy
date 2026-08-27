from core.scenario_manager import *
from core.trigger_condition import *

# for AWSIM
EGO_LANE = '120'
NPC_LANE = '121'

# # for AWSIM-Labs
# EGO_LANE = '111'
# NPC_LANE = '112'

scenario_manager = ScenarioManager()
network = scenario_manager.network
ego = EgoVehicle(init_pose=Pose.from_lane_offset(LaneOffset(EGO_LANE), network),
                 goal_pose=Pose.from_lane_offset(LaneOffset(EGO_LANE, 130), network),
                 speed_limit=30/3.6)
ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))

npc1 = NPCVehicle("npc1", body_style=BodyStyle.HATCHBACK,
                  init_pose=Pose.from_lane_offset(LaneOffset(NPC_LANE, 80), network))
next_lane = network.parse_lane(EGO_LANE)
npc1.add_action(FollowLane(target_speed=10/3.6,
                           condition=av_speed >= 30/3.6-0.1))
npc1.add_action(ChangeLane(next_lane=next_lane,
                           lateral_velocity=1.0,
                           condition=longitudinal_distance_to_ego <= 15))
scenario = Scenario(network, [ego, npc1])
scenario_manager.run([scenario])