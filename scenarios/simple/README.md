## Random Scenarios
### 1. [example1.py](example1.py)
A scenario where an NPC vehicle spawns, accelerates to a target speed, changes lanes at a specified point, and continues following the lane.
```python
scenario_manager = ScenarioManager()
network = scenario_manager.network

_, _, npc_init_pos, npc_init_orient = network.parse_lane_offset(LaneOffset('123'))
npc1 = NPCVehicle("npc1", BodyStyle.SMALL_CAR)
npc1.add_action(SpawnNPCVehicle(position=npc_init_pos, orientation=npc_init_orient))
npc1.add_action(FollowLane(target_speed=20/3.6,
                            acceleration=7))
npc1.add_action(ChangeLane(next_lane=network.parse_lane('124'), lateral_velocity=1.3,
                            condition=reach_point(LaneOffset('123', 12), network)))

scenario = Scenario(network, [npc1])
scenario_manager.run([scenario])
```

The lanes `123` and `124` are shown in the figure below:
![Random Scenario Lanes](../../assets/network.png)

The video below records the scenario execution:



https://github.com/user-attachments/assets/53aa2450-3bb1-4487-85b4-e0758361bcad



