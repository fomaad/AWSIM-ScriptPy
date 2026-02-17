from core.scenario_manager import *
from scenarios.cutin.base import make_cutin_scenario
import argparse

"""
This scenario implements a cut-in behavior for an NPC vehicle.
Used for the Sakura Science Program AD Demo.
"""
def parse_args():
    parser = argparse.ArgumentParser(description='[Sakura Science Program] AD Demo')
    parser.add_argument('--ve', type=float, default=30/3.6, help='ego vehicle speed in m/s (default: 30 km/h)')
    parser.add_argument('--vo', type=float, default=10/3.6, help='npc vehicle speed in m/s (default: 10 km/h)')
    parser.add_argument('--dx0', type=float, default=13, help='initial longitudinal distance between ego and npc in m when the cut-in starts (default: 13 m)')
    parser.add_argument('--vy', type=float, default=1.2, help='lateral cut-in speed of npc in m/s (default: 1.2 m/s)')
    return parser.parse_args()

def make_scenarios(network, ve, vo, dx0, vy):
    scenarios = [
        make_cutin_scenario(network,
                            ego_init_laneoffset=LaneOffset('111', 0),
                            ego_goal_laneoffset=LaneOffset('111', 150),
                            npc_init_laneoffset=LaneOffset('112', 85),
                            cutin_next_lane='111',
                            _ego_speed=ve,
                            _npc_speed=vo,
                            _cutin_vy=vy,
                            dx0=dx0,
                            body_style=BodyStyle.SMALL_CAR,
                            )
    ]
    return scenarios

if __name__ == '__main__':
    args = parse_args()
    scenario_manager = ScenarioManager()
    scenario_manager.run(make_scenarios(scenario_manager.network, args.ve, args.vo, args.dx0, args.vy))