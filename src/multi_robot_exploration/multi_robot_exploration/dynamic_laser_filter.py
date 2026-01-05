#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point
import tf2_ros
import numpy as np
from visualization_msgs.msg import Marker

class DynamicLaserFilter(Node):
    def __init__(self):
        super().__init__('dynamic_laser_filter')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('filtered_scan_topic', '/scan_filtered')
        self.declare_parameter('other_robots', ['TB3_1', 'TB3_2'])
        self.declare_parameter('robot_radius', 0.45)
        self.declare_parameter('self_robot', 'TB3_1')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('world_frame', 'map')

        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.filtered_scan_topic = self.get_parameter('filtered_scan_topic').get_parameter_value().string_value
        self.other_robots = self.get_parameter('other_robots').get_parameter_value().string_array_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self.self_robot = self.get_parameter('self_robot').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"robot_radius utilisé = {self.robot_radius} m")

        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, self.filtered_scan_topic, 10)
        self.marker_publishers = {}
        for robot in self.other_robots:
            topic = f"/{robot}/filter_circle"
            self.marker_publishers[robot] = self.create_publisher(Marker, topic, 1)
            
        # Publisher pour les points filtrés
        self.filtered_points_pub = self.create_publisher(Marker, '/filtered_points', 1)

    def scan_callback(self, msg: LaserScan):        
        try:
            own_tf = self.tf_buffer.lookup_transform(
                self.world_frame, f'{self.self_robot}/{self.base_frame}', rclpy.time.Time())
            own_x = own_tf.transform.translation.x
            own_y = own_tf.transform.translation.y
            # Récupère l’orientation (yaw) du robot
            q = own_tf.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self.get_logger().info(f"[DEBUG] Frame utilisé pour self_robot: world_frame={self.world_frame}, base_frame={self.self_robot}/{self.base_frame}")
        except Exception as e:
            self.get_logger().warn(f'No TF for self: {e}')
            return
        # Get other robots' poses
        robots_xy = []
        for robot in self.other_robots:
            if robot == self.self_robot:
                continue
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.world_frame, f'{robot}/{self.base_frame}', rclpy.time.Time())
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                robots_xy.append((x, y))
                self.get_logger().info(f"[DEBUG] TF trouvée pour {robot}: x={x:.2f}, y={y:.2f} dans frame={self.world_frame}, base_frame={robot}/{self.base_frame}")
                # Publie le cercle de filtrage pour ce robot
                marker = Marker()
                marker.header.frame_id = self.world_frame
                marker.header.stamp = msg.header.stamp
                marker.ns = "filter_circle"
                marker.id = 0
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.05
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = self.robot_radius * 2
                marker.scale.y = self.robot_radius * 2
                marker.scale.z = 0.01
                marker.color.a = 0.5
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                self.marker_publishers[robot].publish(marker)
            except Exception as e:
                self.get_logger().warn(f'No TF for {robot}: {e}')
        if not robots_xy:
            self.get_logger().warn("Aucune position d'autre robot trouvée, aucun filtrage possible !")
        # Filter scan
        filtered = list(msg.ranges)
        angle = msg.angle_min
        nb_filtered = 0
        filtered_points = []
        for i, r in enumerate(msg.ranges):
            if np.isinf(r) or np.isnan(r):
                angle += msg.angle_increment
                continue
            # Point en coordonnées robot
            px = r * np.cos(angle)
            py = r * np.sin(angle)
            # Applique la rotation du robot
            wx = own_x + (np.cos(yaw) * px - np.sin(yaw) * py)
            wy = own_y + (np.sin(yaw) * px + np.cos(yaw) * py)
            # Log la position du point et la distance à chaque robot
            for rx, ry in robots_xy:
                dist = np.hypot(wx - rx, wy - ry)
                if i % 10 == 0:
                    self.get_logger().info(f'[DEBUG] Point {i}: (wx={wx:.2f}, wy={wy:.2f}) dans frame={self.world_frame} vs robot ({rx:.2f},{ry:.2f}) dist={dist:.2f}')
            # Check si dans un robot
            filtered_this = False
            for rx, ry in robots_xy:
                dist = np.hypot(wx - rx, wy - ry)
                if dist < self.robot_radius:
                    filtered[i] = float('nan')
                    filtered_points.append((wx, wy))
                    self.get_logger().info(f'[DEBUG] FILTRAGE: point {i} à (wx={wx:.2f}, wy={wy:.2f}) dans frame={self.world_frame} proche de robot ({rx:.2f},{ry:.2f}), dist={dist:.2f}, rayon={self.robot_radius}')
                    filtered_this = True
                    nb_filtered += 1
                    break
            angle += msg.angle_increment

        # Publie les points filtrés en Marker
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = msg.header.stamp
        marker.ns = "filtered_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.07
        marker.scale.y = 0.07
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.3
        marker.color.b = 1.0
        marker.points = []
        for wx, wy in filtered_points:
            
            p = Point()
            p.x = wx
            p.y = wy
            p.z = 0.05
            marker.points.append(p)
        self.filtered_points_pub.publish(marker)
        self.get_logger().info(f"Points filtrés sur ce scan : {nb_filtered}")
        # Publish
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.ranges = filtered
        filtered_msg.intensities = msg.intensities
        self.scan_pub.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicLaserFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
