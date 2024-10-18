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


class GraphNavInterface:
    def __init__(self, robot, upload_path, use_gps=False):
        self._robot = robot

        # Filepath for uploading a saved graph's and snapshots too.
        if upload_path[-1] == '/':
            self._upload_filepath = upload_path[:-1]
        else:
            self._upload_filepath = upload_path

        self._init_clients()
        self._init_state()

    def _init_clients(self):
        self._robot.time_sync.wait_for_sync()
        self._robot_command_client = self._robot.ensure_client(RobotCommandClient.default_service_name)
        self._robot_state_client = self._robot.ensure_client(RobotStateClient.default_service_name)
        self._graph_nav_client = self._robot.ensure_client(GraphNavClient.default_service_name)
        self._power_client = self._robot.ensure_client(PowerClient.default_service_name)

    def _init_state(self):
        power_state = self._robot_state_client.get_robot_state().power_state
        self._started_powered_on = (power_state.motor_power_state == power_state.STATE_ON)
        self._powered_on = self._started_powered_on
        self._current_graph = None
        self._current_edges = dict()
        self._current_waypoint_snapshots = dict()
        self._current_edge_snapshots = dict()
        self._current_annotation_name_to_wp_id = dict()
        self._spot_power_state = False

    def _get_localization_state(self, *args):
        state = self._graph_nav_client.get_localization_state()
        print(f'Got localization: \n{state.localization}')
        odom_tform_body = get_odom_tform_body(state.robot_kinematics.transforms_snapshot)
        print(f'Got robot state in kinematic odometry frame: \n{odom_tform_body}')

    def _set_initial_localization_fiducial(self, *args):
        robot_state = self._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot).to_proto()
        localization = nav_pb2.Localization()
        self._graph_nav_client.set_localization(initial_guess_localization=localization,
                                                ko_tform_body=current_odom_tform_body)
        
    def _set_initial_localization_waypoint(self, *args):
        """Trigger localization to a waypoint."""
        # Take the first argument as the localization waypoint.
        if len(args) < 1:
            # If no waypoint id is given as input, then return without initializing.
            print('No waypoint specified to initialize to.')
            return
        destination_waypoint = find_unique_waypoint_id(
            args[0][0], self._current_graph, self._current_annotation_name_to_wp_id)
        if not destination_waypoint:
            # Failed to find the unique waypoint id.
            return

        robot_state = self._robot_state_client.get_robot_state()
        current_odom_tform_body = get_odom_tform_body(
            robot_state.kinematic_state.transforms_snapshot).to_proto()
        # Create an initial localization to the specified waypoint as the identity.
        localization = nav_pb2.Localization()
        localization.waypoint_id = destination_waypoint
        localization.waypoint_tform_body.rotation.w = 1.0
        self._graph_nav_client.set_localization(
            initial_guess_localization=localization,
            # It's hard to get the pose perfect, search +/-20 deg and +/-20cm (0.2m).
            max_distance=0.2,
            max_yaw=20.0 * math.pi / 180.0,
            fiducial_init=graph_nav_pb2.SetLocalizationRequest.FIDUCIAL_INIT_NO_FIDUCIAL,
            ko_tform_body=current_odom_tform_body)

    def _list_graph_waypoint_and_edge_ids(self, *args):
        graph = self._graph_nav_client.download_graph()
        if graph is None:
            print('Empty graph.')
            return
        self._current_graph = graph

        localization_id = self._graph_nav_client.get_localization_state().localization.waypoint_id
        
        self._current_annotation_name_to_wp_id, self._current_edges = update_waypoints_and_edges(graph, localization_id)

    def _upload_graph_and_snapshots(self, *args):
        print('Loading the graph from disk into local storage...')
        self._load_graph()
        self._load_waypoints()
        self._load_edges()
        
        print('Uploading the graph and snapshots to the robot...')
        self._upload_graph()
        self._upload_waypoints()
        self._upload_edges()

        self._check_localization()

    def _load_graph(self):
        with open(f'{self._upload_filepath}/graph', 'rb') as graph_file:
            data = graph_file.read()
            self._current_graph = map_pb2.Graph()
            self._current_graph.ParseFromString(data)
            print(f'Loaded graph has {len(self._current_graph.waypoints)} waypoints and {len(self._current_graph.edges)} edges')

    def _load_waypoints(self):
        for waypoint in self._current_graph.waypoints:
            with open(f'{self._upload_filepath}/waypoint_snapshots/{waypoint.snapshot_id}', 'rb') as snapshot_file:
                waypoint_snapshot = map_pb2.WaypointSnapshot()
                waypoint_snapshot.ParseFromString(snapshot_file.read())
                self._current_waypoint_snapshots[waypoint_snapshot.id] = waypoint_snapshot

    def _load_edges(self):
        for edge in self._current_graph.edges:
            if len(edge.snapshot_id) == 0:
                continue
            with open(f'{self._upload_filepath}/edge_snapshots/{edge.snapshot_id}', 'rb') as snapshot_file:
                edge_snapshot = map_pb2.EdgeSnapshot()
                edge_snapshot.ParseFromString(snapshot_file.read())
                self._current_edge_snapshots[edge_snapshot.id] = edge_snapshot

    def _upload_graph(self):
        true_if_empty = not len(self._current_graph.anchoring.anchors)
        response = self._graph_nav_client.upload_graph(graph=self._current_graph,
                                                       generate_new_anchoring=true_if_empty)
        return response

    def _upload_waypoints(self):
        response = self._upload_graph()
        for snapshot_id in response.unknown_waypoint_snapshot_ids:
            waypoint_snapshot = self._current_waypoint_snapshots[snapshot_id]
            self._graph_nav_client.upload_waypoint_snapshot(waypoint_snapshot)
            print(f'Uploaded waypoint snapshot: {waypoint_snapshot.id}')

    def _upload_edges(self):
        response = self._upload_graph()
        for snapshot_id in response.unknown_edge_snapshot_ids:
            edge_snapshot = self._current_edge_snapshots[snapshot_id]
            self._graph_nav_client.upload_edge_snapshot(edge_snapshot)
            print(f'Uploaded edge snapshot: {edge_snapshot.id}')

    def _check_localization(self):
        localization_state = self._graph_nav_client.get_localization_state()
        if not localization_state.localization.waypoint_id:
            print('\nUpload complete! The robot is currently not localized to the map; please localize'
                  ' the robot using commands (2) or (3) before attempting a navigation command.')
            
    def toggle_power(self, should_power_on):
        """Power the robot on/off dependent on the current power state."""
        is_powered_on = self.check_is_powered_on()
        if not is_powered_on and should_power_on:
            # Power on the robot up before navigating when it is in a powered-off state.
            power_on_motors(self._power_client)
            motors_on = False
            while not motors_on:
                future = self._robot_state_client.get_robot_state_async()
                state_response = future.result(
                    timeout=10)  # 10 second timeout for waiting for the state response.
                if state_response.power_state.motor_power_state == robot_state_pb2.PowerState.STATE_ON:
                    motors_on = True
                else:
                    # Motors are not yet fully powered on.
                    time.sleep(.25)
        elif is_powered_on and not should_power_on:
            # Safe power off (robot will sit then power down) when it is in a
            # powered-on state.
            safe_power_off_motors(self._robot_command_client, self._robot_state_client)
        else:
            # Return the current power state without change.
            return is_powered_on
        # Update the locally stored power state.
        self.check_is_powered_on()
        return self._powered_on
    
    def check_is_powered_on(self):
        """Determine if the robot is powered on or off."""
        power_state = self._robot_state_client.get_robot_state().power_state
        self._powered_on = (power_state.motor_power_state == power_state.STATE_ON)
        return self._powered_on

    def _check_success(self, command_id=-1):
        """Use a navigation command id to get feedback from the robot and sit when command succeeds."""
        if command_id == -1:
            # No command, so we have no status to check.
            return False
        status = self._graph_nav_client.navigation_feedback(command_id)
        if status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_REACHED_GOAL:
            # Successfully completed the navigation commands!
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_LOST:
            print('Robot got lost when navigating the route, the robot will now sit down.')
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_STUCK:
            print('Robot got stuck when navigating the route, the robot will now sit down.')
            return True
        elif status.status == graph_nav_pb2.NavigationFeedbackResponse.STATUS_ROBOT_IMPAIRED:
            print('Robot is impaired.')
            return True
        else:
            # Navigation command is not complete yet.
            return False
        
    def _set_upload_filepath(self, upload_path):
        if upload_path[-1] == '/':
            self._upload_filepath = upload_path[:-1]
        else:
            self._upload_filepath = upload_path





"""Graph nav utility functions"""

def id_to_short_code(id):
    """Convert a unique id to a 2 letter short code."""
    tokens = id.split('-')
    if len(tokens) > 2:
        return f'{tokens[0][0]}{tokens[1][0]}'
    return None


def pretty_print_waypoints(waypoint_id, waypoint_name, short_code_to_count, localization_id):
    short_code = id_to_short_code(waypoint_id)
    if short_code is None or short_code_to_count[short_code] != 1:
        short_code = '  '  # If the short code is not valid/unique, don't show it.

    waypoint_symbol = '->' if localization_id == waypoint_id else '  '
    print(
        f'{waypoint_symbol} Waypoint name: {waypoint_name} id: {waypoint_id} short code: {short_code}'
    )


def find_unique_waypoint_id(short_code, graph, name_to_id):
    """Convert either a 2 letter short code or an annotation name into the associated unique id."""
    if graph is None:
        print(
            'Please list the waypoints in the map before trying to navigate to a specific one (Option #4).'
        )
        return

    if len(short_code) != 2:
        # Not a short code, check if it is an annotation name (instead of the waypoint id).
        if short_code in name_to_id:
            # Short code is a waypoint's annotation name. Check if it is paired with a unique waypoint id.
            if name_to_id[short_code] is not None:
                # Has an associated waypoint id!
                return name_to_id[short_code]
            else:
                print(
                    f'The waypoint name {short_code} is used for multiple different unique waypoints. Please use '
                    f'the waypoint id.')
                return None
        # Also not a waypoint annotation name, so we will operate under the assumption that it is a
        # unique waypoint id.
        return short_code

    ret = short_code
    for waypoint in graph.waypoints:
        if short_code == id_to_short_code(waypoint.id):
            if ret != short_code:
                return short_code  # Multiple waypoints with same short code.
            ret = waypoint.id
    return ret


def update_waypoints_and_edges(graph, localization_id, do_print=True):
    """Update and print waypoint ids and edge ids."""
    name_to_id = dict()
    edges = dict()

    short_code_to_count = {}
    waypoint_to_timestamp = []
    for waypoint in graph.waypoints:
        # Determine the timestamp that this waypoint was created at.
        timestamp = -1.0
        try:
            timestamp = waypoint.annotations.creation_time.seconds + waypoint.annotations.creation_time.nanos / 1e9
        except:
            # Must be operating on an older graph nav map, since the creation_time is not
            # available within the waypoint annotations message.
            pass
        waypoint_to_timestamp.append((waypoint.id, timestamp, waypoint.annotations.name))

        # Determine how many waypoints have the same short code.
        short_code = id_to_short_code(waypoint.id)
        if short_code not in short_code_to_count:
            short_code_to_count[short_code] = 0
        short_code_to_count[short_code] += 1

        # Add the annotation name/id into the current dictionary.
        waypoint_name = waypoint.annotations.name
        if waypoint_name:
            if waypoint_name in name_to_id:
                # Waypoint name is used for multiple different waypoints, so set the waypoint id
                # in this dictionary to None to avoid confusion between two different waypoints.
                name_to_id[waypoint_name] = None
            else:
                # First time we have seen this waypoint annotation name. Add it into the dictionary
                # with the respective waypoint unique id.
                name_to_id[waypoint_name] = waypoint.id

    # Sort the set of waypoints by their creation timestamp. If the creation timestamp is unavailable,
    # fallback to sorting by annotation name.
    waypoint_to_timestamp = sorted(waypoint_to_timestamp, key=lambda x: (x[1], x[2]))

    # Print out the waypoints name, id, and short code in an ordered sorted by the timestamp from
    # when the waypoint was created.
    if do_print:
        print(f'{len(graph.waypoints):d} waypoints:')
        for waypoint in waypoint_to_timestamp:
            pretty_print_waypoints(waypoint[0], waypoint[2], short_code_to_count, localization_id)

    # for edge in graph.edges:
    #     if edge.id.to_waypoint in edges:
    #         if edge.id.from_waypoint not in edges[edge.id.to_waypoint]:
    #             edges[edge.id.to_waypoint].append(edge.id.from_waypoint)
    #     else:
    #         edges[edge.id.to_waypoint] = [edge.id.from_waypoint]
    #     if do_print:
    #         print(f'(Edge) from waypoint {edge.id.from_waypoint} to waypoint {edge.id.to_waypoint} '
    #               f'(cost {edge.annotations.cost.value})')

    return name_to_id, edges


def sort_waypoints_chrono(graph):
    """Sort waypoints by time created."""
    waypoint_to_timestamp = []
    for waypoint in graph.waypoints:
        # Determine the timestamp that this waypoint was created at.
        timestamp = -1.0
        try:
            timestamp = waypoint.annotations.creation_time.seconds + waypoint.annotations.creation_time.nanos / 1e9
        except:
            # Must be operating on an older graph nav map, since the creation_time is not
            # available within the waypoint annotations message.
            pass
        waypoint_to_timestamp.append((waypoint.id, timestamp, waypoint.annotations.name))

    # Sort the set of waypoints by their creation timestamp. If the creation timestamp is unavailable,
    # fallback to sorting by annotation name.
    waypoint_to_timestamp = sorted(waypoint_to_timestamp, key=lambda x: (x[1], x[2]))

    return waypoint_to_timestamp
