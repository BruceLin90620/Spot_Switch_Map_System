import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener, TransformException
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3, Pose, PoseWithCovarianceStamped
from std_srvs.srv import Trigger
from std_msgs.msg import Int8
import numpy as np
from transforms3d import quaternions, euler
import yaml
import math

class AprilTagLocalization(Node):
    def __init__(self):
        super().__init__('apriltag_localization')
        
        self._tag_poses_file = '/home/spot/spot_map_switching_ws/src/Spot_Switch_Map_System/spot_graph_nav/tags_pose_data/tags_pose.yaml'
        
        # 定義區域ID到名稱的映射
        self.area_mapping = {
            0: 'Demo_Area',
            1: 'Research_Area',
            2: 'Tea_Room'
        }
        
        self.current_area = 'Research_Area'  # 默認區域
        
        # 創建訂閱者
        self.map_id_sub = self.create_subscription(
            Int8,
            '/map_id',
            self.map_id_callback,
            1
        )
        
        # 加載所有區域的tag資訊
        self.load_all_tag_poses()
        self.setup_tf()

        # 創建 initialpose publisher
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # 創建定時器，每0.5秒檢查一次
        self.pose_timer = self.create_timer(1, self.check_and_publish_pose)

    def check_and_publish_pose(self):
        """
        Periodically check for nearby tags and publish pose if found
        """
        try:
            # Find nearest visible tag
            nearest_tag_id, distance = self.find_visible_tags()
            
            if nearest_tag_id is None:
                return
            
            # 檢查距離是否小於1m
            if distance > 3.0:
                return
                
            # 檢查tag是否在當前區域的信息中
            current_tags = self.all_tag_poses.get(self.current_area, {})
            if nearest_tag_id not in current_tags:
                return

            # 計算並發布位姿
            pose = self.process_detection()
            if pose:
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "map"
                pose_msg.pose.pose = pose
                

                
                self.get_logger().info(f'Publishing pose update with tag {nearest_tag_id} at distance {distance:.2f}m')
                self.pose_publisher.publish(pose_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error in pose check and publish: {str(e)}')

    def find_visible_tags(self):
        """
        Find all visible AprilTags through TF and return the nearest one
        Returns: (tag_id, distance) or (None, None) if no tags are visible
        """
        try:
            # Get all frames in TF tree
            frames = self.tf_buffer.all_frames_as_string()
            
            # Find all fiducial frames
            visible_tags = {}
            now = rclpy.time.Time()
            
            fiducial_frames = []
            for frame in frames.split('\n'):
                if 'fiducial_' in frame and 'filtered' not in frame:
                    # Extract just the frame name (e.g., "fiducial_11")
                    frame_name = frame.split()[1]  # Split by whitespace and take second element
                    if frame_name.startswith('fiducial_'):
                        fiducial_frames.append(frame_name)
            
            # If no fiducial frames found, return None
            if not fiducial_frames:
                self.get_logger().info("No fiducial frames found in TF tree")
                return None, None

            # Print all found fiducial frames
            # self.get_logger().info(f"Found fiducial frames: {fiducial_frames}")
            
            # Get distances to all visible tags
            for frame in fiducial_frames:
                try:
                    # Extract tag ID from frame name
                    tag_id = int(frame.replace('fiducial_', ''))
                    
                    # Get transform from base_link to this fiducial
                    transform = self.tf_buffer.lookup_transform(
                        "base_link",
                        frame,
                        now,
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    
                    # Calculate distance to this tag
                    distance = math.sqrt(
                        transform.transform.translation.x ** 2 +
                        transform.transform.translation.y ** 2 +
                        transform.transform.translation.z ** 2
                    )
                    
                    visible_tags[tag_id] = distance
                    
                except (ValueError, TransformException) as e:
                    self.get_logger().debug(f'Could not get transform for {frame}: {str(e)}')
                    continue
            
            if not visible_tags:
                self.get_logger().warn("No valid transforms to visible tags")
                return None, None
                
            # Find the nearest tag
            nearest_tag_id = min(visible_tags.items(), key=lambda x: x[1])[0]
            nearest_distance = visible_tags[nearest_tag_id]
            
            # self.get_logger().info(
            #     f'Found {len(visible_tags)} tags with valid transforms. '
            #     f'Nearest is tag {nearest_tag_id} at {nearest_distance:.2f}m'
            # )
            
            return nearest_tag_id, nearest_distance
            
        except Exception as e:
            self.get_logger().error(f'Error finding visible tags: {str(e)}')
            return None, None

    def map_id_callback(self, msg):
        """
        Callback for map ID subscription
        """
        map_id = msg.data
        if map_id in self.area_mapping:
            new_area = self.area_mapping[map_id]
            self.current_area = new_area
            self.get_logger().info(f'Switching to area: {self.current_area}')
            self.broadcast_tag_transforms()
        else:
            self.get_logger().warn(f'Received invalid map ID: {map_id}')

            
    def load_all_tag_poses(self):
        """
        Load tag poses for all areas
        """
        try:
            with open(self._tag_poses_file, 'r') as file:
                self.all_tag_poses = yaml.safe_load(file)
            self.get_logger().info("All tag poses loaded successfully")
        except FileNotFoundError:
            self.get_logger().error(f"Error: The file {self._tag_poses_file} was not found.")
            self.all_tag_poses = {}
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML file: {str(e)}")
            self.all_tag_poses = {}

    def setup_tf(self):
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(1.0, self.broadcast_tag_transforms)

    def broadcast_tag_transforms(self):
        # 獲取當前區域的tag信息
        if self.current_area in self.all_tag_poses:
            current_tags = self.all_tag_poses[self.current_area]
            for tag_id, pose in current_tags.items():
                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = 'map'
                transform.child_frame_id = f'tag_{tag_id}'
                transform.transform.translation = Vector3(
                    x=pose['position']['x'],
                    y=pose['position']['y'],
                    z=pose['position']['z']
                )
                transform.transform.rotation = Quaternion(
                    x=pose['orientation']['x'],
                    y=pose['orientation']['y'],
                    z=pose['orientation']['z'],
                    w=pose['orientation']['w']
                )
                self.static_broadcaster.sendTransform(transform)
                self.get_logger().debug(f'Broadcasting transform for tag {tag_id} in {self.current_area}')
        else:
            self.get_logger().warn(f'No tag information found for area: {self.current_area}')

    # def trigger_callback(self, request, response):
    #     """
    #     Service callback for triggering pose publishing
    #     """
    #     try:
    #         pose = self.process_detection()
    #         if pose:

    #             pose_msg = PoseWithCovarianceStamped()
    #             pose_msg.header.stamp = self.get_clock().now().to_msg()
    #             pose_msg.header.frame_id = "map"
    #             pose_msg.pose.pose = pose
                
    #             self.pose_publisher.publish(pose_msg)
                
    #             response.success = True
    #             response.message = "Successfully published pose to /initialpose"
    #         else:
    #             response.success = False
    #             response.message = "Failed to compute pose - no visible tags or transform error"
                
    #     except Exception as e:
    #         response.success = False
    #         response.message = f"Error occurred: {str(e)}"
            
    #     return response

    def process_detection(self):
        try:
            # Find nearest visible tag
            nearest_tag_id, distance = self.find_visible_tags()
            
            if nearest_tag_id is None:
                self.get_logger().warn("No AprilTags visible")
                return None
            
            # 檢查tag是否在當前區域的信息中
            current_tags = self.all_tag_poses.get(self.current_area, {})
            # print(current_tags)
            if nearest_tag_id not in current_tags:
                self.get_logger().warn(f"Tag {nearest_tag_id} not in current area ({self.current_area}) info")
                return None

            # Get transforms for the nearest tag
            base_to_tag = self.tf_buffer.lookup_transform(
                "base_link", 
                f"fiducial_{nearest_tag_id}", 
                rclpy.time.Time()
            )
            
            map_to_tag = self.tf_buffer.lookup_transform(
                "map", 
                f"tag_{nearest_tag_id}", 
                rclpy.time.Time()
            )
            
            robot_pose_map = self.multiply_transforms(
                map_to_tag.transform, 
                self.invert_transform(base_to_tag.transform)
            )
            
            pose = Pose()
            pose.position.x = robot_pose_map.translation.x
            pose.position.y = robot_pose_map.translation.y
            pose.position.z = robot_pose_map.translation.z
            pose.orientation = robot_pose_map.rotation
            
            self.get_logger().info(
                f'Computed pose using tag {nearest_tag_id} at distance {distance:.2f}m: '
                f'x={pose.position.x:.2f}, y={pose.position.y:.2f}, z={pose.position.z:.2f}'
            )
            
            return pose

        except TransformException as ex:
            self.get_logger().error(f'Transform error: {str(ex)}')
            return None
        except Exception as e:
            self.get_logger().error(f'Failed to compute transform: {str(e)}')
            return None

    def multiply_transforms(self, t1, t2):
        m1 = self.transform_to_matrix(t1)
        m2 = self.transform_to_matrix(t2)
        result_matrix = np.dot(m1, m2)
        return self.matrix_to_transform(result_matrix)

    def transform_to_matrix(self, transform):
        translation = [transform.translation.x, transform.translation.y, transform.translation.z]
        rotation = [transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z]
        mat = np.eye(4)
        mat[:3, :3] = quaternions.quat2mat(rotation)
        mat[:3, 3] = translation
        return mat

    def matrix_to_transform(self, matrix):
        translation = matrix[:3, 3]
        rotation_matrix = matrix[:3, :3]
        quat = quaternions.mat2quat(rotation_matrix)
        transform = TransformStamped().transform
        transform.translation = Vector3(x=translation[0], y=translation[1], z=translation[2])
        transform.rotation = Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])
        return transform

    def invert_transform(self, transform):
        mat = self.transform_to_matrix(transform)
        inv_mat = np.linalg.inv(mat)
        return self.matrix_to_transform(inv_mat)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()