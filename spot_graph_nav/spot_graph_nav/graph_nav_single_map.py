import argparse
import logging
import time
import yaml
import sys
import threading

import bosdyn.client
import bosdyn.client.util
from bosdyn.api import geometry_pb2, power_pb2, robot_state_pb2
from bosdyn.api.graph_nav import graph_nav_pb2, map_pb2, nav_pb2
from bosdyn.client.exceptions import ResponseError
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.client.graph_nav import GraphNavClient
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive, ResourceAlreadyClaimedError
from bosdyn.client.math_helpers import Quat, SE3Pose
from bosdyn.client.power import PowerClient, power_on_motors, safe_power_off_motors
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand, blocking_sit
from bosdyn.client.robot_state import RobotStateClient

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from switch_map_interfaces.srv import SingleMap
from switch_map_interfaces.srv import SendGoalPose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

sys.path.append('/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/spot_graph_nav/spot_graph_nav/')
from graph_nav_util import GraphNavInterface
# import graph_nav_util

# map_0 = "/home/spot/graph_nav/maps/cslmap_demo_area.walk"
# map_1 = "/home/spot/graph_nav/maps/cslmap_research_area.walk"
# map_2 = "/home/spot/graph_nav/maps/cslmap_tea_room.walk"
# map_filepath = {0 : "/home/spot/graph_nav/maps/cslmap_demoarea2.walk", 
#                 1 : "/home/spot/graph_nav/maps/cslmap_research_area.walk",
#                 2 : "/home/spot/graph_nav/maps/cslmap_tea_room2.walk"}

# map_filepath = {0 : "/home/spot/spot_map_switching_ws/src/maps/csl_outside_112202.walk", 
#                 1 : "/home/spot/spot_map_switching_ws/src/maps/csl_outside_525354_112202.walk",
#                 2 : "/home/spot/spot_map_switching_ws/src/maps/csl_outside_5355_112201.walk",
#                 3 : "/home/spot/spot_map_switching_ws/src/maps/csl_outside_5456_112201.walk",
#                 4 : "/home/spot/spot_map_switching_ws/src/maps/cslmap_tea_room2.walk",}

class SpotNavigation:
    def __init__(self, robot, upload_path, lease_client, tag_poses_file='/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/spot_graph_nav/tags_pose_data/tags_pose.yaml'):
        self.graph_nav_interface = GraphNavInterface(robot, upload_path)

        self._robot = robot
        self._lease_client = lease_client
        self._upload_filepath = upload_path.rstrip('/')
        self._tag_poses_file = tag_poses_file

        self._init_command_dictionary()
        self._take_lease()
        self._initialize()

    def _init_command_dictionary(self):
        self._command_dictionary = {
            '1': self._initialize,
            '2': self._take_lease,
            '3': self._power_on_and_stand,
            '4': self._power_off_and_sit,
            '5': self._navigate_route,
            '6': self._navigate_to_anchor,
        }

    def _initialize(self, *args):
        while True: 
            try:
                self._clear_graph()
                time.sleep(1)
                self._clear_graph()
                time.sleep(1)
                self.graph_nav_interface._upload_graph_and_snapshots()
                time.sleep(1)
                self.graph_nav_interface._set_initial_localization_fiducial()
                print("initial with apriltag")
                time.sleep(1)
                self.graph_nav_interface._list_graph_waypoint_and_edge_ids()
                print("list finished")
                self.load_tag_poses()
                break

            except Exception as e:
                print(f"Initialization error: {str(e)}")
                while True:
                    choice = input("\nPlease choose:\n1. Reinitialize\n2. Exit program\nEnter (1 or 2): ")
                    if choice == '1':
                        print("\nRestarting initialization...\n")
                        break  
                    elif choice == '2':
                        print("Program terminated")
                        import sys
                        sys.exit(0)
                    else:
                        print("Invalid input, please enter 1 or 2")

    def _take_lease(self, *args):
        self._lease_client.take()
        time.sleep(0.5)

    def _power_on_and_stand(self, *args):
        if not self.graph_nav_interface._powered_on:
            self.graph_nav_interface.toggle_power(should_power_on=True)
        blocking_stand(self.graph_nav_interface._robot_command_client, timeout_sec=10)
        
    def _power_off_and_sit(self, *args):
        blocking_sit(self.graph_nav_interface._robot_command_client, timeout_sec=10)
        if self.graph_nav_interface._powered_on and not self.graph_nav_interface._started_powered_on:
            self.graph_nav_interface.toggle_power(should_power_on=False)

    def load_tag_poses(self):
        try:
            with open(self._tag_poses_file, 'r') as file:
                self.map_tag_info = yaml.safe_load(file)['tag_info']
            print("Tag poses loaded successfully")
        except FileNotFoundError:
            print(f"Error: The file {self._tag_poses_file} was not found.")

    def _navigate_route(self, *args):
        if len(args[0]) < 1:
            print('No waypoint provided as a destination for navigate to.')
            return
        for i in args[0]:
            if int(i) not in self.map_tag_info:
                print(f"Cannot find destination {i}.")
                return
            tag_position = self.map_tag_info[int(i)]
            print(f"tag_position {tag_position}")

            seed_T_goal = SE3Pose(float(tag_position[0]), float(tag_position[1]), 0.0, Quat())

            localization_state = self.graph_nav_interface._graph_nav_client.get_localization_state()
            if not localization_state.localization.waypoint_id:
                print('Robot not localized')
                return
            seed_T_goal.z = localization_state.localization.seed_tform_body.position.z

            seed_T_goal.rot = Quat.from_yaw(float(tag_position[2]))
            
            print(f"seed_T_goal: {seed_T_goal}")

            if not self.graph_nav_interface.toggle_power(should_power_on=True):
                print('Failed to power on the robot, and cannot complete navigate to request.')
                return  

            nav_to_cmd_id = None
            is_finished = False
            while not is_finished:
                try:
                    nav_to_cmd_id = self.graph_nav_interface._graph_nav_client.navigate_to_anchor(
                        seed_T_goal.to_proto(), 1.0, command_id=nav_to_cmd_id)
                except ResponseError as e:
                    print(f'Error while navigating {e}')
                    break
                time.sleep(.5)
                is_finished = self.graph_nav_interface._check_success(nav_to_cmd_id)

            print("mission finish")
            time.sleep(2)

    def _navigate_to_tag(self, *args):
        if len(args[0]) < 1:
            print('No waypoint provided as a destination for navigate to.')
            return

        if int(args[0][0]) not in self.map_tag_info:
            print("Cannot find destination.")
            return

        tag_position = self.map_tag_info[int(args[0][0])]
        print(f"tag_position {tag_position}")

        seed_T_goal = SE3Pose(float(tag_position[0]), float(tag_position[1]), 0.0, Quat())

        localization_state = self.graph_nav_interface._graph_nav_client.get_localization_state()
        if not localization_state.localization.waypoint_id:
            print('Robot not localized')
            return
        seed_T_goal.z = localization_state.localization.seed_tform_body.position.z

        seed_T_goal.rot = Quat.from_yaw(float(tag_position[2]))
        
        print(f"seed_T_goal: {seed_T_goal}")

        if not self.graph_nav_interface.toggle_power(should_power_on=True):
            print('Failed to power on the robot, and cannot complete navigate to request.')
            return

        nav_to_cmd_id = None
        is_finished = False
        while not is_finished:
            try:
                nav_to_cmd_id = self.graph_nav_interface._graph_nav_client.navigate_to_anchor(
                    seed_T_goal.to_proto(), 1.0, command_id=nav_to_cmd_id)
            except ResponseError as e:
                print(f'Error while navigating {e}')
                break
            time.sleep(.5)
            is_finished = self.graph_nav_interface._check_success(nav_to_cmd_id)

    def _navigate_to_anchor(self, *args):
        if len(args) < 1 or len(args[0]) not in [2, 3, 7]:
            print('Invalid arguments supplied.')
            return False

        seed_T_goal = SE3Pose(float(args[0][0]), float(args[0][1]), 0.0, Quat())

        localization_state = self.graph_nav_interface._graph_nav_client.get_localization_state()
        if not localization_state.localization.waypoint_id:
            print('Robot not localized')
            return False
        seed_T_goal.z = localization_state.localization.seed_tform_body.position.z

        if len(args[0]) == 3:
            seed_T_goal.rot = Quat.from_yaw(float(args[0][2]))
        elif len(args[0]) == 7:
            seed_T_goal.rot = Quat(w=float(args[0][3]), x=float(args[0][4]), y=float(args[0][5]),
                                   z=float(args[0][6]))

        if not self.graph_nav_interface.toggle_power(should_power_on=True):
            print('Failed to power on the robot, and cannot complete navigate to request.')
            return False

        nav_to_cmd_id = None
        is_finished = False
        print(seed_T_goal)
        while not is_finished:
            try:
                nav_to_cmd_id = self.graph_nav_interface._graph_nav_client.navigate_to_anchor(
                    seed_T_goal.to_proto(), 1.0, command_id=nav_to_cmd_id)
            except ResponseError as e:
                print(f'Error while navigating {e}')
                return False 
            
            time.sleep(.5)
            is_finished = self.graph_nav_interface._check_success(nav_to_cmd_id)
            
        return True

    def _on_quit(self):
        """Cleanup on quit from the command line interface."""
        # Sit the robot down + power off after the navigation command is complete.
        if self.graph_nav_interface._powered_on and not self.graph_nav_interface._started_powered_on:
            self.graph_nav_interface._robot_command_client.robot_command(RobotCommandBuilder.safe_power_off_command(),
                                                     end_time_secs=time.time())

    def _clear_graph(self, *args):
        """Clear the state of the map on the robot, removing all waypoints and edges."""
        return self.graph_nav_interface._graph_nav_client.clear_graph()

    def run(self):
        """Main loop for the command line interface."""
        while True:
            print("""
            Options:
            (1) Initialize
            (2) Take lease forcefully
            (3) Power on and stand
            (4) Power off and sit
            (5) Navigate along the route
            (q) Exit
            """)
            try:
                inputs = input('>')
            except Exception as e:
                print(e)
                pass

            if inputs is None or len(inputs) == 0:
                print('inputs is None.')
                continue
            
            req_type = str.split(inputs)[0]
            if req_type == 'q':
                self._on_quit()
                break
            
            if req_type not in self._command_dictionary:
                print('Request not in the known command dictionary.')
                continue
            try:
                cmd_func = self._command_dictionary[req_type]
                cmd_func(str.split(inputs)[1:])
            except Exception as e:
                print(e)

class SpotNavigationNode(Node):
    def __init__(self):
        super().__init__('spot_navigation')
        
        self.map_config_path = '/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/config/map_path.yaml'
        self.map_paths = self.load_map_paths()

        self.graph_nav = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.graph_nav_callback, 1)
        self.switch_spot_map_service = self.create_service(SingleMap, '/switch_spot_map', self.switch_map_callback)
        self.publisher_ = self.create_publisher(Marker, '/text_marker', 10)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self._goal_service = self.create_service(SendGoalPose, 'send_goal_pose', self.send_goal_pose_callback, qos_profile = qos)
        # self._action_server = ActionServer(self, SendGoalPose, 'send_goal_pose', self.send_goal_pose_callback)

        # self.graph_nav_target_pose = [0, 0, 0, 0, 0, 0, 0]
        self.map_tag_info = 0
        """Run the command-line interface."""
        parser = argparse.ArgumentParser(description=__doc__)
        bosdyn.client.util.add_base_arguments(parser)
        options = parser.parse_args()

        # Setup and authenticate the robot.
        sdk = bosdyn.client.create_standard_sdk('GraphNavClient')
        robot = sdk.create_robot(options.hostname)
        bosdyn.client.util.authenticate(robot)

        self.lease_client = robot.ensure_client(LeaseClient.default_service_name)

        print(f"Loading initial map: {self.map_paths[0]}")
        self.graph_nav_command_line = SpotNavigation(robot, self.map_paths[0], self.lease_client)

        graph_nav_thread = threading.Thread(target=self.graph_nav_command)
        graph_nav_thread.start()

        self._tag_poses_file = '/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/spot_graph_nav/tags_pose_data/tags_pose.yaml'
        self.load_tag_poses()

        # self.timer = self.create_timer(0.01, self.timer_callback)

    def load_map_paths(self):
        """Load map paths from YAML file"""
        try:
            with open(self.map_config_path, 'r') as file:
                config = yaml.safe_load(file)
                if config["map_paths"]:
                    return config["map_paths"]
                else:
                    self.get_logger().error("No 'map_paths' key found in the configuration file")
                    raise RuntimeError("Invalid configuration file format")
        except Exception as e:
            self.get_logger().error(f"Error reading map paths file: {str(e)}")
            raise

    def load_tag_poses(self):
        try:
            with open(self._tag_poses_file, 'r') as file:
                self.map_tag_info = yaml.safe_load(file)['tag_info']
            print("Tag poses loaded successfully")
        except FileNotFoundError:
            print(f"Error: The file {self._tag_poses_file} was not found.")

    def switch_map_callback(self, request, response):
        if request.mapid < len(self.map_paths):
            self.get_logger().info(f'Changing the map to: {self.map_paths[request.mapid]}')
            self.graph_nav_command_line.graph_nav_interface._set_upload_filepath(self.map_paths[request.mapid])
            response.success = True
            self.graph_nav_command_line._initialize()
        else:
            response.success = False
            self.get_logger().error(f'Invalid map ID: {request.mapid}')
        return response

    def timer_callback(self):
        for tag_id, pose in self.map_tag_info.items():
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'text_marker'
            marker.id = tag_id 
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD

            marker.pose.position = Point(x=pose[0], y=pose[1], z=pose[2])
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0

            marker.color.a = 1.0  
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            marker.text = f'{tag_id}'
            self.publisher_.publish(marker)


    def graph_nav_callback(self, msg):
        graph_nav_target_pose = [0, 0, 0, 0, 0, 0, 0]
        graph_nav_target_pose[0] = msg.pose.position.x
        graph_nav_target_pose[1] = msg.pose.position.y
        graph_nav_target_pose[2] = msg.pose.position.z
        graph_nav_target_pose[3] = msg.pose.orientation.w
        graph_nav_target_pose[4] = msg.pose.orientation.x
        graph_nav_target_pose[5] = msg.pose.orientation.y
        graph_nav_target_pose[6] = msg.pose.orientation.z

        self.get_logger().info(f"graph_nav_target_pose: {graph_nav_target_pose}")

        self.graph_nav_command_line._navigate_to_anchor(graph_nav_target_pose)

    def send_goal_pose_callback(self, request, response):
        graph_nav_target_pose = [0, 0, 0, 0, 0, 0, 0]
        graph_nav_target_pose[0] = request.goal_pose.pose.position.x
        graph_nav_target_pose[1] = request.goal_pose.pose.position.y
        graph_nav_target_pose[2] = request.goal_pose.pose.position.z
        graph_nav_target_pose[3] = request.goal_pose.pose.orientation.w
        graph_nav_target_pose[4] = request.goal_pose.pose.orientation.x
        graph_nav_target_pose[5] = request.goal_pose.pose.orientation.y
        graph_nav_target_pose[6] = request.goal_pose.pose.orientation.z

        self.get_logger().info(f"graph_nav_target_pose: {graph_nav_target_pose}")
        
        success = self.graph_nav_command_line._navigate_to_anchor(graph_nav_target_pose)
        
        if success:
            response.success = True
            response.message = "Goal reached successfully"
            self.get_logger().info('Goal accomplished')
        else:
            response.success = False
            response.message = "Navigation failed"
            self.get_logger().error('Navigation failed')
        
        return response

        

    def graph_nav_command(self):
        print(f"graph nav command line is starting...")
        try:
            with LeaseKeepAlive(self.lease_client, must_acquire=True, return_at_exit=True):
                try:
                    self.graph_nav_command_line.run()
                    return True
                except Exception as exc:  # pylint: disable=broad-except
                    print(exc)
                    print('Graph nav command line client threw an error.')
                    return False
        except ResourceAlreadyClaimedError:
            print(
                'The robot\'s lease is currently in use. Check for a tablet connection or try again in a few seconds.'
            )
            return False


def main():
    rclpy.init()
    print("init finished")
    spot_navigation = SpotNavigationNode()

    # if not estop_gui.main():
    
    rclpy.spin(spot_navigation)
    
    spot_navigation.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    if not main():
        sys.exit(1)