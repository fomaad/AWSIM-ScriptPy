import argparse
from core.scenario_manager import *
from core.trigger_condition import *

"""
This scenario spawns NPCs at random positions and makes them run randomly.
Usage: See `python -m scenarios.random.random_behavior -h`
"""

spawnable_lanes = ['114', '115', '116', '117', '118', '119', '120', '121', '122', '123', '124', '125', '126', '127', '128', '129', '130', 
                   '131', '132', '133', '134', '135', '136', '137', '138', '139', '140', '141', '142', '143', '144', '145', '146', '147', '148', '149']
spawnable_lanes2 = ['243', '244', '245', '406', '1', '47', '48', '49', '93', '94', '95', '96', '84', '85', '86', '217', '218', '496', '498', '499', '183', '184', '185', '186']

def random_pos(number, min_distance=10):
    positions = []
    max_attempts = 1000
    
    for _ in range(number):
        attempts = 0
        while attempts < max_attempts:
            init_lane = random.choice(spawnable_lanes + spawnable_lanes2)
            init_offset = random.uniform(0, 70)
            
            # Check if this position violates minimum distance constraint
            valid = True
            for existing_lane, existing_offset in positions:
                if init_lane == existing_lane:
                    if abs(init_offset - existing_offset) < min_distance:
                        valid = False
                        break
            
            if valid:
                positions.append((init_lane, init_offset))
                break
            
            attempts += 1
        
        # If we couldn't find a valid position after max_attempts, skip this one
        if attempts == max_attempts:
            print(f"Warning: Could not find valid position #{len(positions)+1} after {max_attempts} attempts")
    
    return positions

    
def make_scenario(network, num_npcs=5):
    ego_init_pose = LaneOffset('111')
    ego = EgoVehicle(init_pose=Pose.from_lane_offset(ego_init_pose, network),
                    goal_pose=Pose.from_relative_to_lane(ego_init_pose, network, forward=130))
                    # speed_limit=30/3.6)
    # ego.add_action(Sleep(duration=5))
    ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))

    npcs = []
    init_poses = random_pos(num_npcs)
    for i in range(num_npcs):
        body = random.choice(list(BodyStyle))
        init_lane, init_offset = init_poses[i]
        npc = NPCVehicle(f"npc{i+1}", body,
                         init_pose=Pose.from_lane_offset(LaneOffset(init_lane, init_offset), network))
        # print(f"Spawned NPC {npc.actor_id} at lane {init_lane} with offset {init_offset:.2f}")
        npc.add_action(FollowLane(target_speed=40/3.6))
        for _ in range(20):
            npc.add_action(Sleep(duration=1))
            npc.add_action(RandomAction(network=network))
        npcs.append(npc)

    return Scenario(network, [ego] + npcs)

def parse_args():
    parser = argparse.ArgumentParser(description="Scenario in which NPCs run randomly")
    parser.add_argument('--nonpc', '-n', type=int, default=5, help='Number of NPCs to spawn')
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    scenario_manager = ScenarioManager()
    scenario =  make_scenario(scenario_manager.network, num_npcs=args.nonpc)
    scenario_manager.run([scenario])