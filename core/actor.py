import numpy as np
from core.pose import Pose

class Actor:
    def __init__(self, actor_id, init_pose:Pose=None):
        # static properties
        self.actor_id = actor_id
        self.actions = []
        self.init_pose = init_pose

    def add_action(self, action):
        self.actions.append(action)

    def tick(self, global_state, client_node):
        """
        Execute the actor's actions in order. 
        If an action returns False (not completed), skip the remaining actions until next tick.
        @return: False if no actions were executed, True otherwise
        """
        ids_to_remove = []
        actions_executed = False
        for (i,action) in enumerate(self.actions):
            completed = action.execute(self, global_state, client_node)
            if not completed:
                # skip the remaining actions until next tick
                break
            actions_executed = True
            if completed and action.one_shot:
                ids_to_remove.insert(0, i)

        [self.actions.pop(i) for i in ids_to_remove]
        return actions_executed

class VehicleActor(Actor):
    def __init__(self, actor_id, init_pose:Pose, size=(5,2,1.4), center=(0.0,0.0,0.0)):
        """
        :param size: (length, width, height)
        :param center:
        """
        # static properties
        super().__init__(actor_id, init_pose)
        self.size = np.array(size)
        self.center = np.array(center)
        self.max_speed = 60/3.6
        self.max_acceleration = 9.8

    def get_front_center(self, position, heading_deg):
        """
        :return: the front center point of the vehicle
        """
        front_center_local = self.center[:2] + np.array((self.size[0]/2, 0))
        theta = np.deg2rad(heading_deg)
        rot = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta),  np.cos(theta)]
        ])
        _2dpoint = position[:2] + rot @ front_center_local
        return np.append(_2dpoint, position[2])