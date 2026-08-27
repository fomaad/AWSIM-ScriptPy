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
ego_init_pose = LaneOffset(EGO_LANE)
ego = EgoVehicle(init_pose=Pose.from_lane_offset(ego_init_pose, network),
                 goal_pose=Pose.from_relative_to_lane(ego_init_pose, network, forward=130),
                 speed_limit=30/3.6)
ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))

npc1 = NPCVehicle("npc1", body_style=BodyStyle.HATCHBACK,
                  init_pose=Pose.from_right_lane(ego_init_pose, network, forward=80))
next_lane = network.parse_lane(EGO_LANE)
npc1.add_action(FollowLane(target_speed=10/3.6,
                           condition=av_speed >= 30/3.6-0.1))
npc1.add_action(ChangeLane(next_lane=next_lane,
                           lateral_velocity=1.0,
                           condition=longitudinal_distance_to_ego <= 15))
scenario = Scenario(network, [ego, npc1])
# scenario = Scenario(network, [ego])
scenario_manager.run([scenario])