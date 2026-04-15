from core.actor import *
import utils
import time, json
import std_msgs, rclpy
from core.client_ros_node import *
import enum

class PedestrianStyle(enum.Enum):
    ELEGANT="elegant"
    CASUAL="casual"

class NPCPedestrian(Actor):
    def __init__(self, actor_id, style:PedestrianStyle, 
                 init_pose:Pose=None, pose_callback=None, spawn_condition=None):
        Actor.__init__(self, actor_id, init_pose)
        self.style = style
        self.pose_callback = pose_callback
        self.spawn_condition = spawn_condition
        self.has_spawned = False

    def do_spawn(self, client_node, global_state):
        """
        Spawn the NPC pedestrian in the simulator
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
            "body_style": self.style.value,
            "position": utils.array_to_dict_pos(pos),
            "orientation": utils.array_to_dict_orient(orient)
        }

        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        client_node.dynamic_pedestrian_spawning_publisher.publish(msg)

        # make sure the action was properly handled
        req = DynamicControl.Request()
        req.json_request = msg.data
        retry = 0
        while retry < 10:
            future = client_node.dynamic_pedestrian_spawning_client.call_async(req)
            rclpy.spin_until_future_complete(client_node, future)
            response = future.result()

            if response.status.success:
                client_node.get_logger().info(f"Spawned NPC pedestrian {self.actor_id}")
                self.has_spawned = True
                return
            else:
                # client_node.get_logger().warning(f"Spawning {self.actor_id} was not processed successfully. "
                #                                  f"Retrying... ({retry + 1}/10)")
                time.sleep(client_node.timestep)
                retry += 1
        self.has_spawned = True
        client_node.get_logger().error(f"Sent spawning {self.actor_id} command, but failed to get response after 10 attempts.")