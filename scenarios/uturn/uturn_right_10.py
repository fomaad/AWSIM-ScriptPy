from core.scenario_manager import *
from scenarios.uturn.base import make_uturn_scenario

def make_scenarios(network):
    # fixed params
    vo = 10/3.6

    # dynamic params
    ves = [30, 35, 40]
    dx0s = [26, 31, 36]
    ego_init_laneoffsets = [LaneOffset('514', 30),
                            LaneOffset('514', 17),
                            LaneOffset('282', 4)]
    ego_goal_laneoffsets = [LaneOffset('516', 20),
                            LaneOffset('516', 20),
                            LaneOffset('124', 18)]

    # scenarios
    scenarios = []
    for (ve, dx0, ego_init_laneoffset, ego_goal_laneoffset) in (
            zip(ves, dx0s, ego_init_laneoffsets, ego_goal_laneoffsets)):
        scenarios.append(
            make_uturn_scenario(network,
                                ego_init_laneoffset=ego_init_laneoffset,
                                ego_goal_laneoffset=ego_goal_laneoffset,
                                npc_init_laneoffset=LaneOffset('521', 32),
                                uturn_next_lane='511',
                                _ego_speed=ve/3.6,
                                _npc_speed=vo,
                                dx0=dx0+0.1
                                ))
    return scenarios

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario_manager.run(make_scenarios(scenario_manager.network))