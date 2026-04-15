from core.scenario_manager import *
from scenarios.swerve.base import make_swerve_scenario

def make_scenarios(network):
    # fixed params
    ve = 30/3.6
    vo = 15/3.6

    # dynamic params
    vys = [1.0, 1.2, 1.4]
    dx0s = [35, 29, 25]

    # scenarios
    scenarios = []
    for (vy,dx0) in zip(vys, dx0s):
        scenarios.append(
            make_swerve_scenario(network,
                                 ego_init_laneoffset=LaneOffset('355', 10),
                                 ego_goal_laneoffset=LaneOffset('214', 10),
                                 npc_init_laneoffset=LaneOffset('205', 55),
                                 _ego_speed=ve,
                                 _npc_speed=vo,
                                 body_style=BodyStyle.HATCHBACK,
                                 swerve_vy=vy,
                                 dx0=dx0+0.2
                                ))
    return scenarios

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario_manager.run(make_scenarios(scenario_manager.network))
