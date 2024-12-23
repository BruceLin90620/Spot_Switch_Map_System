#!/usr/bin/env python3

import argparse
import logging
import time
import yaml
import sys
import threading
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from switch_map_interfaces.srv import SingleMap, SendGoalPose
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

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

sys.path.append('/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/spot_graph_nav/spot_graph_nav/')
from graph_nav_util import GraphNavInterface


class SpotNavigation:
    """
    Main class for handling Spot robot navigation using Boston Dynamics' GraphNav system.
    Provides functionality for initialization, movement control, and navigation.
    """
    
    def __init__(self, robot, upload_path: str, lease_client: LeaseClient, 
                 tag_poses_file: str = '/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/spot_graph_nav/tags_pose_data/tags_pose.yaml'):
        self.graph_nav_interface = GraphNavInterface(robot, upload_path)
        self._robot = robot
        self._lease_client = lease_client
        self._upload_filepath = upload_path.rstrip('/')
        self._tag_poses_file = tag_poses_file
        self.initialization_completed = False
        
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
        self.initialization_completed = False
        # while True:
        try:
            # Clear and reload graph
            self._clear_graph()
            time.sleep(1)
            self.graph_nav_interface._upload_graph_and_snapshots()
            time.sleep(1)
            
            # Initialize localization using AprilTags
            self.graph_nav_interface._set_initial_localization_fiducial()
            print("Initial localization with AprilTag complete")
            time.sleep(1)
            
            self.graph_nav_interface._list_graph_waypoint_and_edge_ids()
            print("Waypoint and edge listing complete")
            
            self.load_tag_poses()
            self.initialization_completed = True

        except Exception as e:
            print(f"Initialization error: {str(e)}")
                # while True:
                #     choice = input("\nOptions:\n1. Reinitialize\n2. Exit program\nEnter (1 or 2): ")
                #     if choice == '1':
                #         print("\nRestarting initialization...\n")
                #         break
                #     elif choice == '2':
                #         print("Program terminated")
                #         sys.exit(0)
                #     else:
                #         print("Invalid input, please enter 1 or 2")

    def _take_lease(self, *args):
        self._lease_client.take()
        time.sleep(0.3)

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
        if self.initialization_completed == False:
            print(f"Initialization error: {str(e)}")
            return
        
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
        if self.initialization_completed == False:
            print(f"Initialization error: {str(e)}")
            return
        
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

    def _navigate_to_anchor(self, coords: List[float]) -> bool:
        """
        Navigate robot to a specific anchor point in space.
        
        Args:
            coords: List of coordinates [x, y, z] or [x, y, z, qw, qx, qy, qz]
        
        Returns:
            bool: True if navigation successful, False otherwise
        """
        if len(coords) not in [2, 3, 7]:
            print('Invalid coordinate format supplied.')
            return False

        # Create goal pose
        seed_T_goal = SE3Pose(float(coords[0]), float(coords[1]), 0.0, Quat())
        
        # Get current localization
        localization_state = self.graph_nav_interface._graph_nav_client.get_localization_state()
        if not localization_state.localization.waypoint_id:
            print('Robot not localized')
            return False
            
        # Set z-coordinate and rotation
        seed_T_goal.z = localization_state.localization.seed_tform_body.position.z
        if len(coords) == 3:
            seed_T_goal.rot = Quat.from_yaw(float(coords[2]))
        elif len(coords) == 7:
            seed_T_goal.rot = Quat(w=float(coords[3]), x=float(coords[4]), 
                                 y=float(coords[5]), z=float(coords[6]))

        # Ensure robot is powered on
        if not self.graph_nav_interface.toggle_power(should_power_on=True):
            print('Failed to power on robot')
            return False

        # Execute navigation
        nav_to_cmd_id = None
        is_finished = False
        print(f"Navigation goal: {seed_T_goal}")
        
        while not is_finished:
            try:
                nav_to_cmd_id = self.graph_nav_interface._graph_nav_client.navigate_to_anchor(
                    seed_T_goal.to_proto(), 1.0, command_id=nav_to_cmd_id)
            except ResponseError as e:
                print(f'Navigation error: {e}')
                return False
            
            time.sleep(0.5)
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
    """
    ROS2 Node for handling Spot robot navigation.
    Provides ROS services and subscribers for navigation control.
    """

    def __init__(self):
        super().__init__('spot_graph_nav_node')
        
        # Declare parameters with default values
        self.declare_parameter('map_config_path', 
            '/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/switch_map_system_bringup/config/map_path.yaml')
        self.declare_parameter('tags_poses_file',
            '/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/spot_graph_nav/tags_pose_data/tags_pose.yaml')
        

        self.graph_nav = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.graph_nav_callback, 1)
        self.publisher_ = self.create_publisher(Marker, '/text_marker', 10)

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.switch_spot_map_service = self.create_service(SingleMap, '/switch_spot_map', self.switch_map_callback, qos_profile = qos)
        self._goal_service = self.create_service(SendGoalPose, 'send_goal_pose', self.send_goal_pose_callback, qos_profile = qos)
        # self._init_service = self.create_service(Trigger, '/initialize', self._handle_initialize, qos_profile = qos)
        # self._take_lease_service = self.create_service(Trigger, '/take_lease', self._handle_take_lease, qos_profile = qos)
        # self._power_on_service = self.create_service(Trigger, '/power_on_and_stand', self._handle_power_on_and_stand, qos_profile = qos)
        # self._power_off_service = self.create_service(Trigger, '/power_off_and_sit', self._handle_power_off_and_sit, qos_profile = qos)
        self._init_service = self.create_service(Trigger, '/initialize', self._handle_initialize)
        self._take_lease_service = self.create_service(Trigger, '/take_lease_', self._handle_take_lease)
        self._power_on_service = self.create_service(Trigger, '/power_on_and_stand', self._handle_power_on_and_stand)
        self._power_off_service = self.create_service(Trigger, '/power_off_and_sit', self._handle_power_off_and_sit)


        # Get parameter values
        self.map_config_path = self.get_parameter('map_config_path').get_parameter_value().string_value
        self._tag_poses_file = self.get_parameter('tags_poses_file').get_parameter_value().string_value
        self.map_paths = self.load_map_paths()

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
        current_map = 0
        self.graph_nav_command_line = SpotNavigation(robot, self.map_paths[current_map], self.lease_client)

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


    def _handle_initialize(self, request: Trigger.Request, 
                         response: Trigger.Response) -> Trigger.Response:
        """Handle initialization service call."""
        try:
            self.graph_nav_command_line._initialize()
            response.success = True
            response.message = "Spot initialization completed successfully"
        except Exception as e:
            response.success = False
            response.message = f"Initialization failed: {str(e)}"
            self.get_logger().error(response.message)
        return response
    

    def _handle_take_lease(self, request: Trigger.Request, 
                          response: Trigger.Response) -> Trigger.Response:
        """Handle take lease service call."""
        try:
            self.graph_nav_command_line._take_lease()
            response.success = True
            response.message = "Lease taken successfully"
        except Exception as e:
            response.success = False
            response.message = f"Failed to take lease: {str(e)}"
            self.get_logger().error(response.message)
        return response
    

    def _handle_power_on_and_stand(self, request: Trigger.Request, 
                        response: Trigger.Response) -> Trigger.Response:
        """Handle power on and stand service call."""
        try:
            self.graph_nav_command_line._power_on_and_stand()
            response.success = True
            response.message = "Power on and stand completed successfully"
        except Exception as e:
            response.success = False
            response.message = f"Power off failed: {str(e)}"
            self.get_logger().error(response.message)
        return response


    def _handle_power_off_and_sit(self, request: Trigger.Request, 
                         response: Trigger.Response) -> Trigger.Response:
        """Handle power off and sit service call."""
        try:
            self.graph_nav_command_line._power_off_and_sit()
            response.success = True
            response.message = "Power off and sit completed successfully"
        except Exception as e:
            response.success = False
            response.message = f"Power off failed: {str(e)}"
            self.get_logger().error(response.message)
        return response


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
    spot_graph_nav_node = SpotNavigationNode()
    
    rclpy.spin(spot_graph_nav_node)
    
    spot_graph_nav_node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    if not main():
        sys.exit(1)