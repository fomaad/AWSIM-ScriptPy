from core.scenario_manager import *
from core.trigger_condition import *

def make_deceleration_scenario(network,
                     ego_init_laneoffset,
                     ego_goal_laneoffset,
                     _speed,
                     body_style=BodyStyle.HATCHBACK):

    # ego specification
    ego = EgoVehicle(init_pose=Pose.from_lane_offset(ego_init_laneoffset, network),
                     goal_pose=Pose.from_lane_offset(ego_goal_laneoffset, network),
                     speed_limit=_speed)
    ego.add_action(ActivateAutonomousMode(condition=autonomous_mode_ready()))

    # NPC specification
    # function to calculate the spawn pose of the NPC based on the AV's global state
    def pose_cal(actor, global_state):
        ego_kin = global_state['actor-kinematics']['ego']
        ego_pos = np.array(ego_kin['pose']['position'])
        ego_front_center = actor.get_front_center(ego_pos, ego_kin['pose']['rotation'][2])
        forward = (ego_front_center - ego_pos)[:2]
        forward = forward / np.linalg.norm(forward)
        root_to_back = actor.size[0]/2 - actor.center[0]
        npc_pos = ego_front_center[:2] + (2 * _speed + root_to_back) * forward
        orient = utils.quaternion_from_yaw(ego_kin['pose']['rotation'][2]/180*np.pi)
        return np.append(npc_pos, ego_front_center[2]), orient

    # dynamically spawn the NPC when AV speed >= _speed
    npc1 = NPCVehicle("npc1", body_style,
                      pose_callback=pose_cal,
                      spawn_condition=av_speed >= _speed)

    # Set the moving speed without acceleration step 
    npc1.add_action(FollowLane(target_speed=_speed,
                               acceleration=500))
    npc1.add_action(SetTargetSpeed(target_speed=0,
                                   acceleration=-9.8,
                                   condition=actor_speed >= _speed))

    return Scenario(network, [ego, npc1])

if __name__ == '__main__':
    scenario_manager = ScenarioManager()
    scenario =  make_deceleration_scenario(scenario_manager.network,
                                    ego_init_laneoffset=LaneOffset('111', 0),
                                    ego_goal_laneoffset=LaneOffset('111', 210),
                                    _speed=30 / 3.6
                                    )
    scenario_manager.run([scenario])