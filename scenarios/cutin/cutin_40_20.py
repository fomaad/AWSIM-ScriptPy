from core.scenario_manager import *
from scenarios.cutin.base import make_cutin_scenario

def make_scenarios(network):
    # fixed params
    ego_speed = 40 / 3.6
    npc_speed = 20 / 3.6

    # dynamic params
    vys = [1.2, 1.4, 1.6]
    dx0s = [13, 12, 12]

    # scenarios
    scenarios = []
    for (vy,dx0) in zip(vys, dx0s):
        scenarios.append(
            make_cutin_scenario(network,
                                ego_init_laneoffset=LaneOffset('111', 0),
                                ego_goal_laneoffset=LaneOffset('111', 210),
                                npc_init_laneoffset=LaneOffset('112', 110),
                                cutin_next_lane='111',
                                _ego_speed=ego_speed,
                                _npc_speed=npc_speed,
                                _cutin_vy=vy,
                                dx0=dx0+0.1,
                                body_style=BodyStyle.SMALL_CAR,
                                ))
    return scenarios

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario_manager.run(make_scenarios(scenario_manager.network))