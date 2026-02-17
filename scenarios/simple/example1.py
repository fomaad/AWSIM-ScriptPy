from core.scenario_manager import *
from core.trigger_condition import *

"""
This scenario tests the basic NPC actions: FollowLane, ChangeLane, SetTargetSpeed. 
The NPC starts in lane 123, follows the lane for a while, then changes to lane 124 
when it reaches a certain point. 
You can play with the parameters of the actions and conditions to see different behaviors.
"""
def make_scenario(network):
    npc1 = NPCVehicle("npc1", BodyStyle.SMALL_CAR,
                      init_pose=Pose.from_lane_offset(LaneOffset('123'), network))

    npc1.add_action(FollowLane(target_speed=20/3.6,
                               acceleration=7))
    npc1.add_action(ChangeLane(next_lane=network.parse_lane('124'), lateral_velocity=1.3,
                               condition=reach_point(LaneOffset('123', 12), network)))
    
    # other actions to play with
    # npc1.add_action(SetTargetSpeed(target_speed=2,
    #                                condition=speed >= 15/3.6))
    # npc1.add_action(FollowLane('TrafficLane.335',
    #                            target_speed=3,
    #                            condition=end_lane('TrafficLane.124', network)))

    return Scenario(network, [npc1])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  make_scenario(scenario_manager.network)
    scenario_manager.run([scenario])