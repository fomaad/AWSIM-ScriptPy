from core.scenario_manager import *
from scenarios.uturn.base import make_uturn_scenario

def make_scenarios_10(network):
    vo = 10 / 3.6

    ves = [30, 35, 40]
    dx0s = [26, 30, 36]
    ego_init_laneoffsets = [LaneOffset('132', 20),
                            LaneOffset('132', 12),
                            LaneOffset('303', 1)]
    ego_goal_laneoffsets = [LaneOffset('3001057', 20),
                            LaneOffset('3001057', 20),
                            LaneOffset('134', 18)]

    scenarios = []
    for (ve, dx0, ego_init_laneoffset, ego_goal_laneoffset) in (
            zip(ves, dx0s, ego_init_laneoffsets, ego_goal_laneoffsets)):
        scenarios.append(
            make_uturn_scenario(network,
                                ego_init_laneoffset=ego_init_laneoffset,
                                ego_goal_laneoffset=ego_goal_laneoffset,
                                npc_init_laneoffset=LaneOffset('3001052', 30),
                                uturn_next_lane='132',
                                _ego_speed=ve / 3.6,
                                _npc_speed=vo,
                                dx0=dx0+0.2
                                ))

    return scenarios

def make_scenarios_15(network):
    vo = 15 / 3.6

    ves = [30, 35, 40]
    dx0s = [23, 27, 31]
    ego_init_laneoffsets = [LaneOffset('132', 20),
                            LaneOffset('132', 12),
                            LaneOffset('303', 1)]
    ego_goal_laneoffsets = [LaneOffset('3001057', 20),
                            LaneOffset('3001057', 20),
                            LaneOffset('134', 18)]

    scenarios = []
    for (ve, dx0, ego_init_laneoffset, ego_goal_laneoffset) in (
            zip(ves, dx0s, ego_init_laneoffsets, ego_goal_laneoffsets)):
        scenarios.append(
            make_uturn_scenario(network,
                                ego_init_laneoffset=ego_init_laneoffset,
                                ego_goal_laneoffset=ego_goal_laneoffset,
                                npc_init_laneoffset=LaneOffset('3001052', 30),
                                uturn_next_lane='132',
                                _ego_speed=ve / 3.6,
                                _npc_speed=vo,
                                dx0=dx0-1.8
                                ))
    return scenarios

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenarios = make_scenarios_10(scenario_manager.network) +\
                make_scenarios_15(scenario_manager.network)
    scenario_manager.run(scenarios)
