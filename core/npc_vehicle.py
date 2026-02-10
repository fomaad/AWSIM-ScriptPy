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

class NPCVehicle(VehicleActor):
    def __init__(self, actor_id, body_style:BodyStyle, size=None, center=None,
                 init_pose:Pose=None, pose_callback=None):
        if size is None:
            match body_style:
                case BodyStyle.TAXI:
                    size = (4.64, 1.94, 1.64)
                case BodyStyle.HATCHBACK:
                    size = (4.02,1.94,1.64)
                case BodyStyle.SMALL_CAR:
                    size = (3.48, 1.82, 1.74)
                case BodyStyle.VAN:
                    size = (4.64, 1.94, 1.82)
                case BodyStyle.TRUCK:
                    size = (9.2, 3.08, 4.14)
        if center is None:
            match body_style:
                case BodyStyle.TAXI:
                    center = (1.12, 0.0, 0.85)
                case BodyStyle.HATCHBACK:
                    center = (1.43, 0.0, 0.85)
                case BodyStyle.SMALL_CAR:
                    center = (1.12, 0.0, 0.9)
                case BodyStyle.VAN:
                    center = (1.12, 0.0, 1.18)
                case BodyStyle.TRUCK:
                    center = (0.0, 0.0, 2.13)

        VehicleActor.__init__(self, actor_id, init_pose, size)
        self.body_style = body_style
        self.center = np.array(center)
        self.pose_callback = pose_callback

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
                return
            else:
                # client_node.get_logger().warning(f"Spawning {self.actor_id} was not processed successfully. "
                #                                  f"Retrying... ({retry + 1}/10)")
                time.sleep(client_node.timestep)
                retry += 1
        client_node.get_logger().error(f"Sent spawning {self.actor_id} command, but failed to get response after 10 attempts.")
