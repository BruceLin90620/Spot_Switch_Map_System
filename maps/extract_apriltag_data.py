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

def save_apriltag_data(graphs):
    """
    Extract AprilTag data from multiple graphs and save to a single YAML file.
    :param graphs: List of Graph objects containing AprilTag data
    """
    # Dictionary to store all tag combinations
    tag_combinations = {}
    
    # Process each graph
    for graph in graphs:
        current_tags = {}
        # Extract tag data from graph
        for tag in graph.anchoring.objects:
            pos = tag.seed_tform_object.position
            ori = tag.seed_tform_object.rotation
            
            # Create tag info dictionary
            tag_info = {
                'orientation': {
                    'w': float(ori.w),
                    'x': float(ori.x),
                    'y': float(ori.y),
                    'z': float(ori.z)
                },
                'position': {
                    'x': float(pos.x),
                    'y': float(pos.y),
                    'z': float(pos.z)
                }
            }
            
            tag_id = int(tag.id)
            current_tags[tag_id] = tag_info
            
            # Print tag info to console
            print(f"Tag ID: {tag.id}")
            print(f"Position: x={pos.x:.4f}, y={pos.y:.4f}, z={pos.z:.4f}")
            print(f"Orientation: x={ori.x:.4f}, y={ori.y:.4f}, z={ori.z:.4f}, w={ori.w:.4f}\n")
        
        if current_tags:
            # Generate key for this combination of tags
            tag_key = ''.join(str(tid) for tid in sorted(current_tags.keys()))
            tag_combinations[tag_key] = current_tags

    # Save all combinations to a single YAML file with properly formatted keys
    output_file = 'tags_position.yaml'
    
    # Convert the dictionary to YAML format with custom formatting
    yaml_content = ""
    for tag_key, tag_data in tag_combinations.items():
        yaml_content += f'"{tag_key}":\n'
        # Convert tag data to YAML and indent it properly
        tag_yaml = yaml.dump(tag_data, default_flow_style=False)
        yaml_content += ''.join(f'  {line}\n' for line in tag_yaml.splitlines())
        yaml_content += '\n'  # Add extra newline between sections

    # Write the formatted content to file
    with open(output_file, 'w') as yaml_file:
        yaml_file.write(yaml_content)
    
    print(f"AprilTag data has been saved to {output_file}")

def main(args=None):
    parser = argparse.ArgumentParser(description='Extract AprilTag data from multiple graphs and save to YAML')
    parser.add_argument('paths', type=str, nargs='+', help='Paths to the map directories')
    options = parser.parse_args()

    # Load all maps from the given files
    graphs = [load_map(path) for path in options.paths]
    
    # Extract and save AprilTag data
    save_apriltag_data(graphs)

if __name__ == '__main__':
    main()