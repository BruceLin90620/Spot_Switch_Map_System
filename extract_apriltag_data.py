import argparse
import os
import yaml
from bosdyn.api import geometry_pb2
from bosdyn.api.graph_nav import map_pb2
from bosdyn.client.frame_helpers import *
from bosdyn.client.math_helpers import *

def load_map(path):
    """
    Load a map from the given file path.
    :param path: Path to the root directory of the map.
    :return: the graph
    """
    with open(os.path.join(path, 'graph'), 'rb') as graph_file:
        data = graph_file.read()
        current_graph = map_pb2.Graph()
        current_graph.ParseFromString(data)
        print(f'Loaded graph with {len(current_graph.waypoints)} waypoints and {len(current_graph.anchoring.objects)} anchored world objects')
        return current_graph

def save_apriltag_data(graph, output_file='tags_pose.yaml'):
    """
    Extract AprilTag data from graph and save to YAML file.
    :param graph: Graph object containing AprilTag data
    :param output_file: Path to output YAML file
    """
    # Dictionary to store all tag data organized by areas
    areas_data = {}
    
    # Extract tag data from graph
    for tag in graph.anchoring.objects:
        pos = tag.seed_tform_object.position
        ori = tag.seed_tform_object.rotation
        
        # Create tag info dictionary
        tag_info = {
            'position': {
                'x': float(pos.x),
                'y': float(pos.y),
                'z': float(pos.z)
            },
            'orientation': {
                'x': float(ori.x),
                'y': float(ori.y),
                'z': float(ori.z),
                'w': float(ori.w)
            }
        }
        
        # Determine which area this tag belongs to
        # You can modify this logic based on your needs
        tag_id = int(tag.id)
        if tag_id in [11, 12, 13, 14]:
            area = "Research_Area"
        elif tag_id in [14, 15]:
            area = "Tea_Room"
        elif tag_id in [12, 13, 15]:
            area = "Demo_Area"
        else:
            area = "Other"

        # Initialize area in dictionary if it doesn't exist
        if area not in areas_data:
            areas_data[area] = {}
        
        # Add tag data to appropriate area
        areas_data[area][str(tag.id)] = tag_info
        
        # Print tag info to console
        print(f"Area: {area} | Tag ID: {tag.id}")
        print(f"Position: x={pos.x:.4f}, y={pos.y:.4f}, z={pos.z:.4f}")
        print(f"Orientation: x={ori.x:.4f}, y={ori.y:.4f}, z={ori.z:.4f}, w={ori.w:.4f}\n")

    # Save to YAML file
    with open(output_file, 'w') as yaml_file:
        yaml.dump(areas_data, yaml_file, default_flow_style=False)
    
    print(f"AprilTag data has been saved to {output_file}")

def main(args=None):
    parser = argparse.ArgumentParser(description='Extract AprilTag data from graph and save to YAML')
    parser.add_argument('path', type=str, help='Path to the map directory')
    parser.add_argument('--output', type=str, default='tags_pose.yaml',
                      help='Output YAML file path (default: tags_pose.yaml)')
    options = parser.parse_args()

    # Load the map from the given file
    graph = load_map(options.path)
    
    # Extract and save AprilTag data
    save_apriltag_data(graph, options.output)

if __name__ == '__main__':
    main()