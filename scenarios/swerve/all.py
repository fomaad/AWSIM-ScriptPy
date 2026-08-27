import argparse

from scenarios.swerve.base import *
from core.scenario_manager import ScenarioManager

def make_scenarios_10_30(network):
    ve = 30/3.6
    vo = 10/3.6

    vys = [1.0, 1.2, 1.4]
    dx0s = [35, 32, 31]

    scenarios = []
    for (vy,dx0) in zip(vys, dx0s):
        scenarios.append(
            make_swerve_scenario(network,
                                 ego_init_laneoffset=LaneOffset('227', 1),
                                 ego_goal_laneoffset=LaneOffset('230', 10),
                                 npc_init_laneoffset=LaneOffset('124519', 20),
                                 _ego_speed=ve,
                                 _npc_speed=vo,
                                 swerve_vy=vy,
                                 body_style=BodyStyle.HATCHBACK,
                                 dx0=dx0+0.3,
                                 swerve_dis=5.0,
                                 ))
    return scenarios

def make_scenarios_10_40(network):
    ve = 40/3.6
    vo = 10/3.6

    vys = [1.0, 1.2, 1.4]
    dx0s = [45, 42, 40]

    scenarios = []
    for (vy,dx0) in zip(vys, dx0s):
        scenarios.append(
            make_swerve_scenario(network,
                                 ego_init_laneoffset=LaneOffset('284', 1),
                                 ego_goal_laneoffset=LaneOffset('230', 23),
                                 npc_init_laneoffset=LaneOffset('124519', 40),
                                 _ego_speed=ve,
                                 _npc_speed=vo,
                                 swerve_vy=vy,
                                 body_style=BodyStyle.HATCHBACK,
                                 dx0=dx0+0.3,
                                 swerve_dis=5.0,
                                 ))
    return scenarios

def cli_parser():
    parser = argparse.ArgumentParser(description='Swerve simulation')
    parser.add_argument('-n', type=int, default=1,
                      help='Scenario index (count from 1) to start running. In other words, we ignore ($n-1) first scenarios.')
    return parser

if __name__ == '__main__':
    parser = cli_parser()
    args = parser.parse_args()
    no = args.n - 1
    if no == 0:
        no = 0
    scenario_manager = ScenarioManager()
    scenarios = (
        make_scenarios_10_30(scenario_manager.network) +
        make_scenarios_10_40(scenario_manager.network)
    )
    scenario_manager.run(scenarios[no:])