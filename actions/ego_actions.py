import time

from core.action import Action
import utils

from geometry_msgs.msg import Pose, PoseWithCovariance, PoseWithCovarianceStamped
# from tier4_planning_msgs.msg import VelocityLimit
from autoware_internal_planning_msgs.msg import VelocityLimit

class SpawnEgo(Action):
    def __init__(self, position, orientation, condition=None):
        super().__init__(condition=condition, one_shot=True)
        self.position = position
        self.orientation = orientation

    def _do(self, actor, client_node, global_state):
        """
        Spawn the ego vehicle and request (re-)localization
        :param position:
        :param orientation:
        :return:
        """
        ros_pose = utils.obj_to_ros_pose(self.position, self.orientation)
        cov = [0.0] * 36
        cov[0] = 1e-4  # x
        cov[7] = 1e-4  # y
        cov[35] = 1e-4  # yaw

        # publish a pose message
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = client_node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose = ros_pose
        msg.pose.covariance = cov

        client_node.ego_pose_publisher.publish(msg)
        time.sleep(1)

        ok = client_node.re_localization(ros_pose, cov)
        if ok:
            client_node.get_logger().info("Spawned ego and re-localized successfully.")
            # re-localization succeeded
            # self.ads_internal_status = AdsInternalStatus.GOAL_SET
            # self.loop()
        else:
            client_node.get_logger().error("Failed to localize ego.")

class SetGoalPose(Action):
    def __init__(self, position, orientation, condition=None):
        super().__init__(one_shot=True, condition=condition)
        self.position = position
        self.orientation = orientation

    def _do(self, actor, client_node, global_state):
        """
        Set goal for autonomous driving
        :param position:
        :param orientation:
        :return:
        """
        time.sleep(1)
        ros_goal = utils.obj_to_ros_pose(self.position, self.orientation)
        client_node.set_goal(ros_goal)

class ActivateAutonomousMode(Action):
    def __init__(self, condition=None, callback=None):
        super().__init__(one_shot=True, condition=condition, callback=callback)

    def _do(self, actor, client_node, global_state):
        client_node.send_engage_cmd()

class SetVelocityLimit(Action):
    def __init__(self, max_velocity, condition=None, one_shot=False):
        """
        :param max_velocity: m/s unit
        :param condition:
        """
        super().__init__(one_shot=one_shot, condition=condition)
        self.max_velocity = max_velocity

    def _do(self, actor, client_node, global_state):
        vel_limit_msg = VelocityLimit()
        vel_limit_msg.max_velocity = float(self.max_velocity)
        vel_limit_msg.use_constraints = False
        vel_limit_msg.sender = ""
        client_node.ego_max_speed_publisher.publish(vel_limit_msg)

