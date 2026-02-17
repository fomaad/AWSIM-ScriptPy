import time
from core.actor import *
import utils

from geometry_msgs.msg import PoseWithCovarianceStamped
try:
    from tier4_planning_msgs.msg import VelocityLimit
except ImportError:
    from autoware_internal_planning_msgs.msg import VelocityLimit

class EgoVehicle(VehicleActor):
    def __init__(self, init_pose:Pose, goal_pose:Pose, 
                 speed_limit=None,
                 size=(4.886,2.186,1.421), center=(1.424,0.0,0.973)):
        VehicleActor.__init__(self, "ego", init_pose, size)
        self.center = np.array(center)
        self.goal_pose = goal_pose
        self.speed_limit = speed_limit

    def do_spawn(self, client_node):
        """
        Spawn the ego vehicle and request (re-)localization
        """
        ros_pose = utils.obj_to_ros_pose(self.init_pose.position, self.init_pose.orientation)
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

    def do_set_goal(self, client_node):
        """
        Set goal for autonomous driving
        """
        time.sleep(1)
        ros_goal = utils.obj_to_ros_pose(self.goal_pose.position, self.goal_pose.orientation)
        client_node.set_goal(ros_goal)
    
    def do_set_speed_limit(self, client_node):
        if self.speed_limit is None:
            return
        vel_limit_msg = VelocityLimit()
        vel_limit_msg.max_velocity = float(self.speed_limit)
        vel_limit_msg.use_constraints = False
        vel_limit_msg.sender = ""
        client_node.ego_max_speed_publisher.publish(vel_limit_msg)