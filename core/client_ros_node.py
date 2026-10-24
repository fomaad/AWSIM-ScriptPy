import time, os, threading
from map.network import *

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import std_msgs.msg
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseWithCovariance
from aw_monitor.srv import *
from autoware_adapi_v1_msgs.srv import InitializeLocalization, ChangeOperationMode, ClearRoute
from autoware_adapi_v1_msgs.msg import RouteState
from autoware_vehicle_msgs.msg import Engage
import utils
try:
    from autoware_internal_planning_msgs.msg import VelocityLimit
except ImportError:
    from tier4_planning_msgs.msg import VelocityLimit
import aw_monitor.msg

current_dir = os.path.dirname(os.path.abspath(__file__))

MOTION_STATE_UNKNOWN = 0
MOTION_STATE_STOPPED = 1
MOTION_STATE_STARTING = 2
MOTION_STATE_MOVING = 3

ROUTING_STATE_UNKNOWN = 0
ROUTING_STATE_UNSET = 1
ROUTING_STATE_SET = 2
ROUTING_STATE_ARRIVED = 3
ROUTING_STATE_CHANGING = 4

# OPERATION_STATE_UNKNOWN = 0
# OPERATION_STATE_STOP = 1
# OPERATION_STATE_AUTONOMOUS = 2
# OPERATION_STATE_LOCAL = 3
# OPERATION_STATE_REMOTE = 4

AWSIM_CLIENT_OP_STATE_STOPPED = 1
AWSIM_CLIENT_OP_STATE_RUNNING = 2
AWSIM_CLIENT_OP_STATE_AUTO_MODE = 3

class AdsInternalStatus(Enum):
    UNINITIALIZED = 0
    LOCALIZATION_SUCCEEDED = 1
    GOAL_SET = 2
    AUTONOMOUS_MODE_READY = 3
    AUTONOMOUS_IN_PROGRESS = 4
    GOAL_ARRIVED = 5

    def __lt__(self, other):
        if isinstance(other, AdsInternalStatus):
            return self.value < other.value
        return NotImplemented
    def __le__(self, other):
        if isinstance(other, AdsInternalStatus):
            return self.value <= other.value
        return NotImplemented

    def __ge__(self, other):
        if isinstance(other, AdsInternalStatus):
            return self.value >= other.value
        return NotImplemented
    def __gt__(self, other):
        if isinstance(other, AdsInternalStatus):
            return self.value > other.value
        return NotImplemented

class ClientNode(Node):
    def __init__(self):
        super().__init__('awsimscript_client')
        self.timestep = 0.05
        self.ads_internal_status = AdsInternalStatus.UNINITIALIZED
        self.ego_motion_state = MOTION_STATE_STOPPED
        self.published_finish_signal = False

        # Subscriptions and service clients live in different callback groups, so a pending
        # service response is never blocked behind a subscription callback (and vice versa).
        self._sub_cb_group = MutuallyExclusiveCallbackGroup()
        self._srv_cb_group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # publishers
        self.awsim_scenario_publisher = self.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/script/awsim_script',
            qos_profile)

        # Ego's goal setting
        self.ego_goal_publisher = self.create_publisher(
            PoseStamped,
            '/planning/mission_planning/goal',
            qos_profile
        )
        # autonomous engage command
        self.ego_auto_engage_publisher = self.create_publisher(
            Engage,
            '/autoware/engage',
            qos_profile
        )
        # to despawn NPCs
        self.npc_removing_publisher = self.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/npc/remove',
            qos_profile
        )
        # to publish signals when simulation starts, finishes
        self.client_op_status_publisher = self.create_publisher(
            std_msgs.msg.Int32,
            '/awsim_script_client/scenario_op_status',
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )
        )
        self.ego_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos_profile
        )
        # ROS publisher for max velocity setting
        self.ego_max_speed_publisher = self.create_publisher(
            VelocityLimit,
            '/planning/scenario_planning/max_velocity',
            qos_profile
        )
        # dynamically spawning NPC vehicles
        self.dynamic_npc_spawning_publisher = self.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/vehicle/spawn',
            qos_profile
        )

        # follow lane command
        self.follow_lane_publisher = self.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/vehicle/follow_lane',
            qos_profile
        )

        # follow waypoints command
        self.follow_waypoints_publisher = self.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/vehicle/follow_waypoints',
            qos_profile
        )

        # set target speed
        self.set_target_speed_publisher = self.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/vehicle/target_speed',
            qos_profile
        )

        # spawn NPC pedestrians
        self.dynamic_pedestrian_spawning_publisher = self.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/pedestrian/spawn',
            qos_profile
        )
        # pedestrian follow waypoints command
        self.ped_follow_waypoints_publisher = self.create_publisher(
            std_msgs.msg.String,
            '/dynamic_control/pedestrian/follow_waypoints',
            qos_profile
        )

        # service clients
        # execute a script
        self.awsim_scenario_client = self.create_client(
            DynamicControl,
            '/dynamic_control/script/awsim_script_srv',
        )

        # localization initilization
        self.init_localization_request = self.create_client(
            InitializeLocalization,
            '/api/localization/initialize'
        )

        # execution state of AWSIM and Autoware, include: motion state, routing state, operation state
        self.execution_state_client = self.create_client(
            ExecutionState,
            '/simulation/gt_srv/execution_state'
        )

        # recording state of AW-RuntimeMonitor (e.g., recording, trace writing, trace written, etc.)
        self.recording_state_client = self.create_client(
            MonitorRecordingState,
            '/monitor/recording/state'
        )
        # groundtruth states of all actors
        self.groundtruth_kinematic_client = self.create_client(
            GroundtruthKinematic,
            '/simulation/gt_srv/kinematic'
        )
        self.groundtruth_size_client = self.create_client(
            GroundtruthSize,
            '/simulation/gt_srv/size'
        )
        # to clear route
        self.clear_route_client = self.create_client(
            ClearRoute,
            '/api/routing/clear_route'
        )
        # to despawn NPCs
        self.npc_removing_client = self.create_client(
            DynamicControl,
            '/dynamic_control/npc/remove_srv',
        )

        self.map_network_client = self.create_client(
            DynamicControl,
            '/dynamic_control/map/network',
        )
        self.dynamic_npc_spawning_client = self.create_client(
            DynamicControl,
            '/dynamic_control/vehicle/spawn_srv',
        )
        self.follow_lane_client = self.create_client(
            DynamicControl,
            '/dynamic_control/vehicle/follow_lane_srv',
        )
        self.follow_waypoints_client = self.create_client(
            DynamicControl,
            '/dynamic_control/vehicle/follow_waypoints_srv',
        )
        self.set_target_speed_client = self.create_client(
            DynamicControl,
            '/dynamic_control/vehicle/target_speed_srv',
        )
        # pedestrians
        self.dynamic_pedestrian_spawning_client = self.create_client(
            DynamicControl,
            '/dynamic_control/pedestrian/spawn_srv',
        )
        self.ped_follow_waypoints_client = self.create_client(
            DynamicControl,
            '/dynamic_control/pedestrian/follow_waypoints_srv',
        )

        # Subscriptions for state updates. The callbacks only store the latest message.
        self._execution_state = None
        self._kinematics = None
        self._actor_sizes = None

        self.execution_state_sub = self.create_subscription(
            aw_monitor.msg.ExecutionState,
            '/simulation/gt/execution_state',
            self._execution_state_callback,
            10,
            callback_group=self._sub_cb_group
        )
        
        self.kinematics_sub = self.create_subscription(
            aw_monitor.msg.GroundtruthKinematic,
            '/simulation/gt/kinematic',
            self._gtkinematic_callback,
            10,
            callback_group=self._sub_cb_group
        )
        
        self.sizes_sub = self.create_subscription(
            aw_monitor.msg.GroundtruthSize,
            '/simulation/gt/size',
            self._gtsize_callback,
            10,
            callback_group=self._sub_cb_group
        )

        # The node is spin continuously in a background thread, so the caller's thread is free
        # to block (e.g., waiting for a service response) without losing any message.
        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def create_client(self, srv_type, srv_name, **kwargs):
        kwargs.setdefault('callback_group', self._srv_cb_group)
        return super().create_client(srv_type, srv_name, **kwargs)

    def destroy_node(self):
        self._executor.shutdown()
        self._spin_thread.join(timeout=5.0)
        super().destroy_node()

    def _execution_state_callback(self, msg):
        with self._state_lock:
            self._execution_state = msg
    
    def _gtkinematic_callback(self, msg):
        with self._state_lock:
            self._kinematics = msg

    def _gtsize_callback(self, msg):
        with self._state_lock:
            self._actor_sizes = msg

    def get_execution_state(self):
        with self._state_lock:
            return self._execution_state
    
    def get_kinematics(self):
        with self._state_lock:
            return self._kinematics
    
    def get_actor_sizes(self):
        with self._state_lock:
            return self._actor_sizes

    def wait_for_state(self, timeout_sec=5.0):
        """
        Wait until the execution state and the groundtruth kinematics have been received.
        :return: True if both were received before the timeout.
        """
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self.get_execution_state() is not None and self.get_kinematics() is not None:
                return True
            time.sleep(0.01)
        return False

    def call_service(self, client, req, timeout_sec=10.0, wait_for_service_sec=5.0):
        """
        Call a service and block the calling thread until the response arrives.
        The background executor completes the future, so this must not spin the node.
        :return: the response, or None if the service is unavailable, timed out, or failed.
        """
        if not client.wait_for_service(timeout_sec=wait_for_service_sec):
            self.get_logger().warn(f"Service {client.srv_name} is not available.")
            return None

        future = client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout_sec):
            client.remove_pending_request(future)
            self.get_logger().warn(f"Service {client.srv_name} timed out after {timeout_sec}s.")
            return None
        if future.exception() is not None:
            self.get_logger().warn(f"Service {client.srv_name} failed: {future.exception()}")
            return None
        return future.result()

    def send_request(self, file_path):
        self.publish_start_signal()

        while not self.awsim_scenario_client.wait_for_service(timeout_sec=5.0):
            print('[WARNING] /dynamic_control/script/awsim_script_srv ROS service not available, waiting...')

        my_dict = {
            "file": file_path,
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        self.awsim_scenario_publisher.publish(msg)

        self.get_logger().info(f'Sent request: {file_path}')

        time.sleep(1)

        req = DynamicControl.Request()
        req.json_request = msg.data
        response = self.call_service(self.awsim_scenario_client, req)
        if response is None:
            self.get_logger().error("No response from AWSIM for the script file request.")
        elif response.status.success:
            self.get_logger().info(f"Script file {file_path} was processed properly.")
            try:
                initpose_and_goal = json.loads(response.status.message)
                ros_pose = utils.dict_to_ros_pose(initpose_and_goal['initial_pose']['pose'])
                if self.re_localization(ros_pose, initpose_and_goal['initial_pose']['covariance']):
                    # re-localization succeeded
                    ros_goal = utils.dict_to_ros_pose(initpose_and_goal['goal'])
                    self.set_goal(ros_goal)
                    self.ads_internal_status = AdsInternalStatus.GOAL_SET
                    self.loop()

            except json.decoder.JSONDecodeError:
                self.get_logger().error(f"Ego initial pose and goal are unknown. Message: {response.status.message}")
        else:
            self.get_logger().error(f"AWSIM failed to process the script file, "
                  f"error message: {response.status.message}.")

    def re_localization(self, ros_pose, covariance=None):
        """
        :param ros_pose: instance of geometry_msgs.msg.Pose
        :param covariance:
        :return:
        """
        while not self.init_localization_request.wait_for_service(timeout_sec=5.0):
            print('[WARNING] Localization initialization ROS service not available, waiting...')

        req = InitializeLocalization.Request()
        req.pose.append(PoseWithCovarianceStamped())
        req.pose[0].header.frame_id = 'map'

        req.pose[0].pose.pose = ros_pose
        if not covariance:
            covariance = [0.0] * 36
            covariance[0] = 1e-4   # x
            covariance[7] = 1e-4   # y
            covariance[35] = 1e-4  # yaw
        req.pose[0].pose.covariance = covariance

        retry = 0
        while retry < 10:
            req.pose[0].header.stamp = self.get_clock().now().to_msg()
            response = self.call_service(self.init_localization_request, req)

            if response is not None and response.status.success:
                self.get_logger().info("Re-Localization succeeded.")
                return True
            else:
                message = "no response" if response is None else response.status.message
                self.get_logger().warn(f"Re-Localization failed, message: {message}")
                self.get_logger().info("Retrying localization...")
                time.sleep(1)
                retry += 1
        self.get_logger().error(f"Localization failed.")
        return False

    def set_goal(self, ros_goal):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose = ros_goal
        self.ego_goal_publisher.publish(msg)
        self.get_logger().info('Setting Ego destination done')

    def send_engage_cmd(self):
        msg = Engage()
        msg.stamp = self.get_clock().now().to_msg()
        msg.engage = True

        self.ego_auto_engage_publisher.publish(msg)
        self.get_logger().info("Autonomous mode activated.")
        self.ads_internal_status = AdsInternalStatus.AUTONOMOUS_IN_PROGRESS
        # self.publish_in_auto_mode_signal()

    def upd_execution_state(self):
        response = self.query_execution_state()
        if response is None:
            return

        self.ego_motion_state = response.motion_state
        if response.is_autonomous_mode_available and \
                self.ads_internal_status < AdsInternalStatus.AUTONOMOUS_MODE_READY:
            self.get_logger().info("Autonomous operation mode is ready")
            self.ads_internal_status = AdsInternalStatus.AUTONOMOUS_MODE_READY
            self.send_engage_cmd()
            self.publish_in_auto_mode_signal()

        if (response.routing_state == ROUTING_STATE_ARRIVED and
                self.ads_internal_status == AdsInternalStatus.AUTONOMOUS_IN_PROGRESS):
            self.get_logger().info("Arrived destination")
            self.ads_internal_status = AdsInternalStatus.GOAL_ARRIVED

    def loop(self):
        while self.ads_internal_status < AdsInternalStatus.AUTONOMOUS_IN_PROGRESS:
            self.upd_execution_state()
            time.sleep(1)

    def query_execution_state(self):
        while not self.execution_state_client.wait_for_service(timeout_sec=5.0):
            print('[WARNING] Execution state ROS service not available, waiting...')

        # Create a request
        req = ExecutionState.Request()
        return self.call_service(self.execution_state_client, req)

    def query_recording_state(self):
        while not self.recording_state_client.wait_for_service(timeout_sec=5.0):
            print('[WARNING] Monitor Recording state ROS service not available, waiting...')

        req = MonitorRecordingState.Request()
        return self.call_service(self.recording_state_client, req)

    def clear_route(self):
        while not self.clear_route_client.wait_for_service(timeout_sec=5.0):
            print('[WARNING] Route clearing ROS service not available, waiting...')

        req = ClearRoute.Request()
        response = self.call_service(self.clear_route_client, req)
        if response is None:
            self.get_logger().warn("Route clearing: no response")
            return
        self.get_logger().info(f"Route clearing: {response.status.success}, {response.status.message}")

    def remove_npcs(self):
        my_dict = {
            "target": "", #empty target means removing all NPC vehicles and pedestrians
        }
        msg = std_msgs.msg.String()
        msg.data = json.dumps(my_dict)
        self.npc_removing_publisher.publish(msg)

        # do a service request to confirm the despawning
        req = DynamicControl.Request()
        req.json_request = msg.data
        retry = 0
        response = None
        while retry < 10:
            response = self.call_service(self.npc_removing_client, req)
            if response is not None and response.status.success:
                self.get_logger().info(f"NPCs removed.")
                break

            time.sleep(self.timestep)
            retry += 1

        if retry >= 10:
            message = "no response" if response is None else response.status.message
            self.get_logger().warn(f"Failed to remove NPC vehicle(s), error message: {message}")

    def publish_start_signal(self):
        # publish running signal
        msg = std_msgs.msg.Int32()
        msg.data = AWSIM_CLIENT_OP_STATE_RUNNING
        self.client_op_status_publisher.publish(msg)

    def publish_in_auto_mode_signal(self):
        msg = std_msgs.msg.Int32()
        msg.data = AWSIM_CLIENT_OP_STATE_AUTO_MODE
        self.client_op_status_publisher.publish(msg)

    def publish_finish_signal(self):
        if not self.published_finish_signal:
            msg = std_msgs.msg.Int32()
            msg.data = AWSIM_CLIENT_OP_STATE_STOPPED
            self.client_op_status_publisher.publish(msg)
            self.published_finish_signal = True

    def query_groundtruth_kinematics(self):
        while not self.groundtruth_kinematic_client.wait_for_service(timeout_sec=5.0):
            print('[WARNING] Ground truth kinematic ROS service not available, waiting...')

        # Create a request
        req = GroundtruthKinematic.Request()
        return self.call_service(self.groundtruth_kinematic_client, req)

    def query_groundtruth_size(self):
        while not self.groundtruth_size_client.wait_for_service(timeout_sec=5.0):
            print('[WARNING] Ground truth kinematic ROS service not available, waiting...')

        # Create a request
        req = GroundtruthSize.Request()
        return self.call_service(self.groundtruth_size_client, req)

    def send_map_network_req(self):
        """
        :return: instance of network.Network
        """
        while not self.map_network_client.wait_for_service(timeout_sec=2.0):
            print('[WARNING] Map network ROS service not available, waiting...')

        req = DynamicControl.Request()
        req.json_request = ""
        self.get_logger().info(f"Sending Map Network Request")
        response = None
        while response is None:
            response = self.call_service(self.map_network_client, req, timeout_sec=30.0)
        self.get_logger().info(f"Map Network Response: {response.status.success}")

        return Network(response.status.message)
