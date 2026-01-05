#!/usr/bin/env python3
"""
ICP pour fusion multi-robots :
- On fixe TB3_1 à (0,0,0) dans un repère arbitraire (origine de la fusion)
- On exprime la pose de TB3_2 par rapport à TB3_1 grâce à l'ICP (nuage2 -> nuage1)
- On écrit le YAML pour map_merge avec ces poses
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import sys
import time
import math
import numpy as np
import open3d as o3d
from tf2_ros import Buffer, TransformListener
import yaml

class ICPAlignment(Node):
    def __init__(self):
        super().__init__('icp_alignment')
        self.pc1 = None
        self.pc2 = None
        self.pc1_received = False
        self.pc2_received = False
        self.frames_to_accumulate = 1
        self.buffers = {'TB3_1': [], 'TB3_2': []}
        # Cache TF de 30 secondes pour avoir suffisamment d'historique
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, qos=QoSProfile(depth=10))
        
        # Attendre 5 secondes pour que le TF buffer se remplisse
        self.get_logger().info('Attente de 5s pour que le TF buffer se remplisse...')
        time.sleep(5.0)
        
        self.sub1 = self.create_subscription(PointCloud2, '/TB3_1/camera_depth/points', self.pc1_callback, 10)
        self.sub2 = self.create_subscription(PointCloud2, '/TB3_2/camera_depth/points', self.pc2_callback, 10)
        self.get_logger().info('Attente des nuages de points TB3_1 et TB3_2...')

    def pc1_callback(self, msg):
        self.ingest_pointcloud(msg, 'TB3_1')
    def pc2_callback(self, msg):
        self.ingest_pointcloud(msg, 'TB3_2')

    def ingest_pointcloud(self, msg, robot_name):
        buffer = self.buffers[robot_name]
        if len(buffer) >= self.frames_to_accumulate:
            return
        pcd = self.ros_to_open3d(msg, robot_name)
        if pcd is None or len(pcd.points) == 0:
            return
        buffer.append(pcd)
        self.get_logger().info(f'{robot_name}: frame {len(buffer)}/{self.frames_to_accumulate}')
        if len(buffer) == self.frames_to_accumulate:
            processed = self.merge_and_preprocess(buffer)
            if robot_name == 'TB3_1':
                self.pc1 = processed
                self.pc1_received = True
            else:
                self.pc2 = processed
                self.pc2_received = True
            if self.pc1_received and self.pc2_received:
                self.try_icp()

    def ros_to_open3d(self, msg, robot_name):
        points = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p[0], p[1], p[2]
            dist = np.sqrt(x*x + y*y + z*z)
            if 0.2 < dist < 4.0 and abs(z) < 1.5:
                points.append([x, y, z])
        if len(points) == 0:
            return None
        points_array = np.array(points)
        # TRANSFORMATION VERS LE REPERE MAP
        target_frame = f'{robot_name}/map'
        source_frame = msg.header.frame_id
        # Utiliser le timestamp du message pour le lookup TF
        timestamp = msg.header.stamp
        try:
            # Vérifier d'abord avec le timestamp du message
            if not self.tf_buffer.can_transform(target_frame, source_frame, timestamp, timeout=Duration(seconds=2.0)):
                # Si ça échoue, test avec le dernier TF disponible
                self.get_logger().warn(f'TF au timestamp du message non disponible, essai avec dernier TF...')
                if not self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=2.0)):
                    self.get_logger().warn(f'TF non disponible: {source_frame} -> {target_frame}')
                    return None
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=2.0))
            else:
                transform = self.tf_buffer.lookup_transform(target_frame, source_frame, timestamp, timeout=Duration(seconds=2.0))
        except Exception as e:
            self.get_logger().warn(f'Erreur TF {source_frame} -> {target_frame}: {e}')
            return None
        t = transform.transform.translation
        r = transform.transform.rotation
        translation = np.array([t.x, t.y, t.z])
        rotation = self.quat_to_matrix(r.x, r.y, r.z, r.w)
        points_transformed = (rotation @ points_array.T).T + translation
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_transformed)
        return pcd

    @staticmethod
    def quat_to_matrix(x, y, z, w):
        norm = math.sqrt(x*x + y*y + z*z + w*w)
        if norm < 1e-10:
            return np.eye(3)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ])

    def merge_and_preprocess(self, clouds):
        merged = o3d.geometry.PointCloud()
        for cloud in clouds:
            merged += cloud
        merged = merged.voxel_down_sample(0.02)
        merged, _ = merged.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        return merged

    def try_icp(self):
        self.get_logger().info('Lancement ICP...')
        pc1 = self.pc1.voxel_down_sample(0.03)
        pc2 = self.pc2.voxel_down_sample(0.03)
        pc1.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        pc2.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        fpfh1 = o3d.pipelines.registration.compute_fpfh_feature(pc1, o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=100))
        fpfh2 = o3d.pipelines.registration.compute_fpfh_feature(pc2, o3d.geometry.KDTreeSearchParamHybrid(radius=0.25, max_nn=100))
        result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
            pc2, pc1, fpfh2, fpfh1, mutual_filter=True, max_correspondence_distance=0.3,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
            ransac_n=4,
            checkers=[
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.3)
            ],
            criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 0.9999)
        )
        result_icp = o3d.pipelines.registration.registration_icp(
            pc2, pc1, 0.1, result_ransac.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200)
        )
        final_result = result_icp if result_icp.fitness > result_ransac.fitness * 0.5 else result_ransac
        T = final_result.transformation
        dx = T[0, 3]
        dy = T[1, 3]
        R = T[:3, :3]
        yaw_detected = math.atan2(R[1, 0], R[0, 0])
        
        yaw = 0.0
        
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('RESULTAT: Pose de TB3_2 relative à TB3_1 (repère arbitraire)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'   x = {dx:.4f} m')
        self.get_logger().info(f'   y = {dy:.4f} m')
        self.get_logger().info(f'   yaw_detected = {yaw_detected:.4f} rad ({math.degrees(yaw_detected):.1f} deg) [IGNORÉ]')
        self.get_logger().info(f'   Fitness: {final_result.fitness:.2f}')
        self.get_logger().info(f'   RMSE: {final_result.inlier_rmse:.3f} m')
        config = {
            'map_merge': {
                'ros__parameters': {
                    'known_init_poses': True,
                    'merging_rate': 2.0,
                    'world_frame': 'world',
                    'robot_map_topic': 'map',
                    'robot_namespace': 'TB3',
                    '/TB3_1/map_merge/init_pose_x': 0.0,
                    '/TB3_1/map_merge/init_pose_y': 0.0,
                    '/TB3_1/map_merge/init_pose_z': 0.0,
                    '/TB3_1/map_merge/init_pose_yaw': 0.0,
                    '/TB3_2/map_merge/init_pose_x': float(dx),
                    '/TB3_2/map_merge/init_pose_y': float(dy),
                    '/TB3_2/map_merge/init_pose_z': 0.0,
                    '/TB3_2/map_merge/init_pose_yaw': float(yaw)
                }
            }
        }
        output_file = os.path.expanduser('~/ros2_humble/src/multi_robot_exploration/config/map_merge_icp_poses.yaml')
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        with open(output_file, 'w') as f:
            yaml.dump(config, f, default_flow_style=False)
        self.get_logger().info(f'Sauvegarde: {output_file}')
        self.get_logger().info('TERMINE')
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = ICPAlignment()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
