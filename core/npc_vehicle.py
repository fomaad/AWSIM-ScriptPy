from core.actor import *
import utils
import time, json
import std_msgs, rclpy
from core.client_ros_node import *
import enum

class BodyStyle(enum.Enum):
    TAXI="taxi"
    HATCHBACK="hatchback"
    SMALL_CAR="small-car"
    VAN="van"
    TRUCK="truck"

veh_sizes = {
    "taxi" : [4.64, 1.94, 1.64],  # taxi
    "hatchback" : [4.02, 1.94, 1.64],  # hatchback
    "small-car" : [3.48, 1.82, 1.74],  # small car
    "van" : [4.64, 1.94, 1.82],  # van
    "truck" : [9.2, 3.08, 4.14],   # truck
}
veh_center_offsets = {
    "taxi" : [1.12, 0.0, 0.85],  # taxi
    "hatchback" : [1.43, 0.0, 0.85],  # hatchback
    "small-car" : [1.12, 0.0, 0.9],  # small car
    "van" : [1.12, 0.0, 1.18],  # van
    "truck" : [0.0, 0.0, 2.13],   # truck
}

class NPCVehicle(VehicleActor):
    def __init__(self, actor_id, body_style:BodyStyle, size=None, center=None,
                 init_pose:Pose=None, pose_callback=None, spawn_condition=None):
        if size is None:
            size = veh_sizes.get(body_style.value)
        if center is None:
            center = veh_center_offsets.get(body_style.value)

        VehicleActor.__init__(self, actor_id, init_pose, size)
        self.body_style = body_style
        self.center = np.array(center)
        self.pose_callback = pose_callback
        self.spawn_condition = spawn_condition
        self.has_spawned = False

    def do_spawn(self, client_node, global_state):
        """
        Spawn the NPC vehicle in the simulator
        :param client_node:
        :return:
        """
        if self.pose_callback is not None:
            pos, orient = self.pose_callback(self, global_state)
        else:
            pos = self.init_pose.position
            orient = self.init_pose.orientation
        my_dict = {
            "timestamp": client_node.get_clock().now().nanoseconds / 1e9,
            "name": self.actor_id,
            "body_style": self.body_style.value,
            "position": utils.array_to_dict_pos(pos),
            "orientation": utils.array_to_dict_orient(orient)
        }

        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        client_node.dynamic_npc_spawning_publisher.publish(msg)

        # make sure the action was properly handled
        req = DynamicControl.Request()
        req.json_request = msg.data
        retry = 0
        while retry < 10:
            future = client_node.dynamic_npc_spawning_client.call_async(req)
            rclpy.spin_until_future_complete(client_node, future)
            response = future.result()

            if response.status.success:
                client_node.get_logger().info(f"Spawned NPC vehicle {self.actor_id}")
                self.has_spawned = True
                return
            else:
                # client_node.get_logger().warning(f"Spawning {self.actor_id} was not processed successfully. "
                #                                  f"Retrying... ({retry + 1}/10)")
                time.sleep(client_node.timestep)
                retry += 1
        self.has_spawned = True
        client_node.get_logger().error(f"Sent spawning {self.actor_id} command, but failed to get response after 10 attempts.")

    @staticmethod
    def get_size(body_style: BodyStyle):
        return veh_sizes.get(body_style.value)
    @staticmethod
    def get_center_offset(body_style: BodyStyle):
        return veh_center_offsets.get(body_style.value)