from core.scenario_manager import *
from scenarios.swerve.base import make_swerve_scenario

def make_scenarios(network):
    # fixed params
    ve = 40/3.6
    vo = 10/3.6

    # dynamic params
    vys = [1.0, 1.2, 1.4]
    # dx0s = [39, 34, 30]
    dx0s = [44, 40, 38]
    offsets = [57, 62, 66]

    # scenarios
    scenarios = []
    for (vy,dx0,offset) in zip(vys, dx0s,offsets):
        scenarios.append(
            make_swerve_scenario(network,
                                 ego_init_laneoffset=LaneOffset('268'),
                                 ego_goal_laneoffset=LaneOffset('214', 26),
                                 npc_init_laneoffset=LaneOffset('205', offset),
                                 _ego_speed=ve,
                                 _npc_speed=vo,
                                 swerve_vy=vy,
                                 body_style=BodyStyle.HATCHBACK,
                                 dx0=dx0+0.2,
                                 swerve_dis=4.0,
                                ))
    return scenarios

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario_manager.run(make_scenarios(scenario_manager.network))
