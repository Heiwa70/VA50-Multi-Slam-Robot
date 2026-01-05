#!/usr/bin/env python3
"""
Frontier Search Optimisé pour exploration multi-robot
- Précision : Tolérance à 0.3m pour forcer le robot à aller VRAIMENT au bout (après test)
- Vision : Dilatation augmentée (x5) pour détecter les frontières même s'il y a des trous dans la map.
"""

import warnings
warnings.filterwarnings("ignore", message=".*NumPy version.*")

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import cv2
from dataclasses import dataclass, field
from typing import List, Dict, Optional
import threading
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

FREE_SPACE = 0
UNKNOWN_SPACE = -1
LETHAL_OBSTACLE = 100
GOAL_TOLERANCE = 0.3
FORCE_RESEND_TIME = 5.0

@dataclass
class Frontier:
    size: int = 0
    centroid: Point = field(default_factory=Point)
    goal_point: Point = field(default_factory=Point)
    points: List[Point] = field(default_factory=list)
    area_m2: float = 0.0

class FastFrontierSearch:
    def __init__(self, min_frontier_size: float = 0.2, max_frontiers: int = 50):
        self.min_frontier_size = min_frontier_size
        self.max_frontiers = max_frontiers
        self.costmap_data = None
        self.width = 0
        self.height = 0
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.costmap_frame = ""
        self.lock = threading.Lock()
        self.first_costmap = True
    
    def update_costmap(self, costmap_msg: OccupancyGrid):
        with self.lock:
            self.costmap_data = np.array(costmap_msg.data, dtype=np.int8).reshape(
                (costmap_msg.info.height, costmap_msg.info.width))
            self.width = costmap_msg.info.width
            self.height = costmap_msg.info.height
            self.resolution = costmap_msg.info.resolution
            self.origin_x = costmap_msg.info.origin.position.x
            self.origin_y = costmap_msg.info.origin.position.y
            self.costmap_frame = costmap_msg.header.frame_id
            
            if self.first_costmap:
                self.first_costmap = False
                unique, counts = np.unique(self.costmap_data, return_counts=True)
                stats = dict(zip(unique, counts))
                return stats
            return None
    
    def map_to_world(self, mx: int, my: int) -> tuple[float, float]:
        wx = self.origin_x + (mx + 0.5) * self.resolution
        wy = self.origin_y + (my + 0.5) * self.resolution
        return wx, wy
    
    def find_frontiers_fast(self, debug: bool = False) -> List[Frontier]:
        with self.lock:
            if self.costmap_data is None:
                return []
            
            costmap = self.costmap_data.copy()
            
            # Masques de base
            free_mask = (costmap == 0).astype(np.uint8)
            unknown_mask = (costmap == -1).astype(np.uint8)
            # Inclure aussi les zones à faible coût comme navigables
            navigable_mask = ((costmap >= 0) & (costmap < 80)).astype(np.uint8)
            
            if debug:
                print(f"[DEBUG] Free: {np.sum(free_mask)}, Unknown: {np.sum(unknown_mask)}, Navigable: {np.sum(navigable_mask)}")
            
            if np.sum(unknown_mask) == 0:
                if debug:
                    print("[DEBUG] Aucune zone inconnue!")
                return []
            
            # Essayer plusieurs niveaux de dilatation pour trouver TOUTES les frontières
            all_frontier_masks = []
            
            # Dilatation faible
            kernel_small = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            free_expanded_small = cv2.dilate(free_mask, kernel_small, iterations=2)
            # Ajouter les zones navigables dilatées
            navigable_expanded = cv2.dilate(navigable_mask, kernel_small, iterations=2)
            accessible_small = cv2.bitwise_or(free_expanded_small, navigable_expanded)
            accessible_unknown_small = cv2.bitwise_and(unknown_mask, accessible_small)
            all_frontier_masks.append(accessible_unknown_small)
            
            # Dilatation moyenne
            kernel_medium = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            free_expanded_medium = cv2.dilate(free_mask, kernel_medium, iterations=3)
            accessible_unknown_medium = cv2.bitwise_and(unknown_mask, free_expanded_medium)
            all_frontier_masks.append(accessible_unknown_medium)
            
            # Dilatation forte
            kernel_large = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            free_expanded_large = cv2.dilate(free_mask, kernel_large, iterations=5)
            accessible_unknown_large = cv2.bitwise_and(unknown_mask, free_expanded_large)
            all_frontier_masks.append(accessible_unknown_large)
            
            # Combiner toutes les détections
            accessible_unknown = np.zeros_like(unknown_mask)
            for mask in all_frontier_masks:
                accessible_unknown = cv2.bitwise_or(accessible_unknown, mask)
            
            if debug:
                print(f"[DEBUG] Accessible unknown pixels: {np.sum(accessible_unknown)}")
            
            if np.sum(accessible_unknown) == 0:
                if debug:
                    print("[DEBUG] Aucune zone inconnue accessible!")
                return []
            
            # Détection des bords de frontière
            kernel_edge = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
            unknown_eroded = cv2.erode(accessible_unknown, kernel_edge, iterations=1)
            unknown_border = accessible_unknown - unknown_eroded
            
            # détecter la frontière comme l'interface free/unknown
            # Dilater free et voir où ça touche unknown
            free_dilated = cv2.dilate(free_mask, kernel_edge, iterations=1)
            direct_frontier = cv2.bitwise_and(unknown_mask, free_dilated)
            
            # Combiner les deux méthodes
            frontier_mask = cv2.bitwise_or(unknown_border, direct_frontier)
            
            # Nettoyage
            kernel_clean = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            frontier_mask = cv2.morphologyEx(frontier_mask, cv2.MORPH_CLOSE, kernel_clean)
            # Dilater légèrement pour connecter les points proches
            frontier_mask = cv2.dilate(frontier_mask, kernel_clean, iterations=1)
            
            if debug:
                print(f"[DEBUG] Frontier mask pixels: {np.sum(frontier_mask)}")
            
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
                frontier_mask, connectivity=8
            )
            
            if debug:
                print(f"[DEBUG] Nombre de composantes: {num_labels - 1}")
            
            frontiers = []
            # Reduit le seuil adaptatif pour ne pas rater les petites frontières
            adaptive_threshold = self.min_frontier_size * 0.3
            if num_labels < 5:
                adaptive_threshold = max(0.02, self.min_frontier_size * 0.2)
            
            for label in range(1, num_labels):
                area_pixels = stats[label, cv2.CC_STAT_AREA]
                frontier_length_m = area_pixels * self.resolution
                
                if debug and frontier_length_m >= 0.01:
                    print(f"[DEBUG] Frontière {label}: {frontier_length_m:.2f}m (seuil: {adaptive_threshold:.2f}m)")
                
                if frontier_length_m < adaptive_threshold:
                    continue
                
                cx_map = int(centroids[label, 0])
                cy_map = int(centroids[label, 1])
                frontier_points_idx = np.argwhere(labels == label)
                
                if len(frontier_points_idx) > 0:
                    points_xy = frontier_points_idx[:, [1, 0]] 
                    centroid_xy = np.array([[cx_map, cy_map]])
                    dists = cdist(centroid_xy, points_xy)
                    costs = costmap[points_xy[:, 1], points_xy[:, 0]]
                    
                    # Chercher un point de goal navigable
                    mask_super_safe = (costs >= 0) & (costs < 10)
                    mask_safe = (costs >= 0) & (costs < 50)
                    mask_ok = (costs >= 0) & (costs < 80)
                    
                    gx_map, gy_map = -1, -1
                    
                    if np.any(mask_super_safe):
                        valid_points = points_xy[mask_super_safe]
                        valid_dists = dists[0][mask_super_safe]
                        gx_map, gy_map = valid_points[np.argmin(valid_dists)]
                    elif np.any(mask_safe):
                        valid_points = points_xy[mask_safe]
                        valid_dists = dists[0][mask_safe]
                        gx_map, gy_map = valid_points[np.argmin(valid_dists)]
                    elif np.any(mask_ok):
                        valid_points = points_xy[mask_ok]
                        valid_dists = dists[0][mask_ok]
                        gx_map, gy_map = valid_points[np.argmin(valid_dists)]
                    else:
                        # chercher le point FREE le plus proche du centroid
                        gx_map, gy_map = self._find_nearest_free_point(cx_map, cy_map, free_mask)
                        if gx_map == -1:
                            if debug: 
                                print(f"[DEBUG] Frontière {label} rejetée: pas de point navigable")
                            continue
                else:
                    gx_map, gy_map = cx_map, cy_map

                if not (0 <= gx_map < self.width and 0 <= gy_map < self.height):
                    continue

                cx_world, cy_world = self.map_to_world(cx_map, cy_map)
                gx_world, gy_world = self.map_to_world(gx_map, gy_map)
                
                frontier = Frontier()
                frontier.size = area_pixels
                frontier.area_m2 = frontier_length_m
                frontier.centroid = Point(x=cx_world, y=cy_world, z=0.0)
                frontier.goal_point = Point(x=gx_world, y=gy_world, z=0.0)
                
                step = max(1, len(frontier_points_idx) // 20)
                for py, px in frontier_points_idx[::step]:
                    if costmap[py, px] < 100:
                        wx, wy = self.map_to_world(px, py)
                        frontier.points.append(Point(x=wx, y=wy, z=0.0))
                
                # accepter les frontières avec moins de points
                if len(frontier.points) >= 1:
                    frontiers.append(frontier)
                    if debug:
                        print(f"[DEBUG] ✓ Frontière {label} acceptée: ({gx_world:.1f}, {gy_world:.1f})")
                
                if len(frontiers) >= self.max_frontiers:
                    break
            
            frontiers.sort(key=lambda f: f.area_m2, reverse=True)
            
            if debug:
                print(f"[DEBUG] Total frontières trouvées: {len(frontiers)}")
            
            return frontiers
    
    def _find_nearest_free_point(self, cx: int, cy: int, free_mask: np.ndarray) -> tuple[int, int]:
        """Trouve le point libre le plus proche du centroid."""
        # Chercher dans un rayon croissant
        for radius in range(1, 50):
            y_min = max(0, cy - radius)
            y_max = min(self.height, cy + radius + 1)
            x_min = max(0, cx - radius)
            x_max = min(self.width, cx + radius + 1)
            
            region = free_mask[y_min:y_max, x_min:x_max]
            free_points = np.argwhere(region > 0)
            
            if len(free_points) > 0:
                # Trouver le plus proche
                free_points_global = free_points + np.array([y_min, x_min])
                dists = np.sqrt((free_points_global[:, 0] - cy)**2 + 
                               (free_points_global[:, 1] - cx)**2)
                closest_idx = np.argmin(dists)
                gy, gx = free_points_global[closest_idx]
                return int(gx), int(gy)
        
        return -1, -1
        
class OptimalFrontierAllocator:
    def __init__(self, distance_weight: float = 1.0, size_weight: float = 0.5):
        self.distance_weight = distance_weight
        self.size_weight = size_weight
        self.hysteresis_radius = 5.0
        self.hysteresis_bonus = 30.0
        self.min_robot_separation = 3.0  # Distance minimale entre les goals (en mètres)
        self.min_goal_to_robot_dist = 2.5  # Distance minimale entre un goal et l'AUTRE robot
        self.separation_penalty = 500.0  #  Pénalité forte
    
    def compute_cost(self, frontier: Frontier, robot_pos: Point) -> float:
        distance = np.sqrt(
            (frontier.goal_point.x - robot_pos.x)**2 + 
            (frontier.goal_point.y - robot_pos.y)**2
        )
        return self.distance_weight * distance - self.size_weight * frontier.area_m2
    
    def allocate(self, frontiers: List[Frontier], 
                 robot_positions: Dict[str, Point],
                 current_robot_goals: Dict[str, Optional[Point]] = None) -> Dict[str, Optional[Frontier]]:
        
        if not frontiers or not robot_positions:
            return {ns: None for ns in robot_positions.keys()}
        
        robot_names = list(robot_positions.keys())
        allocation = {ns: None for ns in robot_names}
        unassigned_robots = list(robot_names)

        if not unassigned_robots:
            return allocation

        n_u_robots = len(unassigned_robots)
        n_frontiers = len(frontiers)
        cost_matrix = np.full((n_u_robots, n_frontiers), np.inf)  # Commencer avec inf
        
        for i, robot_ns in enumerate(unassigned_robots):
            robot_pos = robot_positions[robot_ns]
            prev_goal = current_robot_goals.get(robot_ns) if current_robot_goals else None
            
            for j, frontier in enumerate(frontiers):
                # Vérifier d'abord si cette frontière est trop proche d'un autre robot
                too_close_to_other = False
                for other_ns, other_pos in robot_positions.items():
                    if other_ns != robot_ns and other_pos is not None:
                        dist_to_other_robot = np.sqrt(
                            (frontier.goal_point.x - other_pos.x)**2 + 
                            (frontier.goal_point.y - other_pos.y)**2
                        )
                        if dist_to_other_robot < self.min_goal_to_robot_dist:
                            too_close_to_other = True
                            break
                
                # Si trop proche d'un autre robot, coût infini (frontière interdite)
                if too_close_to_other:
                    continue  # Laisse le coût à inf
                
                base_cost = self.compute_cost(frontier, robot_pos)
                
                if prev_goal:
                    dist_to_prev = np.sqrt((frontier.goal_point.x - prev_goal.x)**2 + 
                                           (frontier.goal_point.y - prev_goal.y)**2)
                    if dist_to_prev < self.hysteresis_radius:
                        base_cost -= self.hysteresis_bonus
                
                dist_now = np.sqrt((frontier.goal_point.x - robot_pos.x)**2 + 
                                   (frontier.goal_point.y - robot_pos.y)**2)
                if dist_now < 0.4:
                    base_cost += 50.0 
                
                # Pénalité graduée pour proximité avec l'autre robot
                for other_ns, other_pos in robot_positions.items():
                    if other_ns != robot_ns and other_pos is not None:
                        dist_to_other_robot = np.sqrt(
                            (frontier.goal_point.x - other_pos.x)**2 + 
                            (frontier.goal_point.y - other_pos.y)**2
                        )
                        if dist_to_other_robot < self.min_robot_separation * 1.5:
                            proximity_factor = 1.0 - (dist_to_other_robot / (self.min_robot_separation * 1.5))
                            base_cost += self.separation_penalty * proximity_factor
                
                cost_matrix[i, j] = base_cost
        
        # Vérifier qu'il y a des solutions valides
        valid_assignments_exist = np.any(np.isfinite(cost_matrix))
        if not valid_assignments_exist:
            # utiliser les coûts de base sans les contraintes strictes
            for i, robot_ns in enumerate(unassigned_robots):
                robot_pos = robot_positions[robot_ns]
                for j, frontier in enumerate(frontiers):
                    cost_matrix[i, j] = self.compute_cost(frontier, robot_pos)
        
        # Remplacer les inf par une grande valeur pour l'algorithme hongrois
        max_finite = np.max(cost_matrix[np.isfinite(cost_matrix)]) if np.any(np.isfinite(cost_matrix)) else 1000
        cost_matrix = np.where(np.isinf(cost_matrix), max_finite * 10, cost_matrix)
        
        row_indices, col_indices = linear_sum_assignment(cost_matrix)
        
        # Allocation avec vérification stricte de séparation
        allocated_goals = []
        
        # Trier par coût pour donner priorité aux meilleures allocations
        assignments = list(zip(row_indices, col_indices))
        assignments.sort(key=lambda x: cost_matrix[x[0], x[1]])
        
        for r_idx, f_idx in assignments:
            robot_ns = unassigned_robots[r_idx]
            frontier = frontiers[f_idx]
            goal = frontier.goal_point
            robot_pos = robot_positions[robot_ns]
            
            # Vérifier que ce goal est assez loin de tous les goals déjà assignés
            valid = True
            for other_ns, other_goal in allocated_goals:
                dist_between_goals = np.sqrt(
                    (goal.x - other_goal.x)**2 + 
                    (goal.y - other_goal.y)**2
                )
                if dist_between_goals < self.min_robot_separation:
                    valid = False
                    break
            
            # Vérifier aussi que ce goal ne va pas croiser le chemin d'un autre robot
            for other_ns, other_pos in robot_positions.items():
                if other_ns != robot_ns and other_pos is not None:
                    # Vérifier la distance goal -> autre robot
                    dist_goal_to_other = np.sqrt(
                        (goal.x - other_pos.x)**2 + 
                        (goal.y - other_pos.y)**2
                    )
                    if dist_goal_to_other < self.min_goal_to_robot_dist:
                        valid = False
                        break
            
            if valid:
                allocation[robot_ns] = frontier
                allocated_goals.append((robot_ns, goal))
            else:
                # Chercher une frontière alternative
                best_alternative = self._find_alternative_frontier(
                    robot_ns, robot_pos, frontiers, frontier, 
                    allocated_goals, robot_positions
                )
                if best_alternative is not None:
                    allocation[robot_ns] = best_alternative
                    allocated_goals.append((robot_ns, best_alternative.goal_point))
        
        return allocation
    
    def _find_alternative_frontier(self, robot_ns: str, robot_pos: Point,
                                   frontiers: List[Frontier], 
                                   excluded_frontier: Frontier,
                                   allocated_goals: List,
                                   robot_positions: Dict[str, Point]) -> Optional[Frontier]:
        """Trouve une frontière alternative qui respecte les contraintes de séparation."""
        best_alternative = None
        best_cost = float('inf')
        
        for alt_frontier in frontiers:
            if alt_frontier == excluded_frontier:
                continue
            
            alt_goal = alt_frontier.goal_point
            
            # Vérifier distance avec les goals déjà assignés
            far_from_goals = True
            for _, assigned_goal in allocated_goals:
                dist = np.sqrt(
                    (alt_goal.x - assigned_goal.x)**2 + 
                    (alt_goal.y - assigned_goal.y)**2
                )
                if dist < self.min_robot_separation:
                    far_from_goals = False
                    break
            
            if not far_from_goals:
                continue
            
            # Vérifier distance avec les autres robots
            far_from_robots = True
            for other_ns, other_pos in robot_positions.items():
                if other_ns != robot_ns and other_pos is not None:
                    dist = np.sqrt(
                        (alt_goal.x - other_pos.x)**2 + 
                        (alt_goal.y - other_pos.y)**2
                    )
                    if dist < self.min_goal_to_robot_dist:
                        far_from_robots = False
                        break
            
            if far_from_robots:
                cost = self.compute_cost(alt_frontier, robot_pos)
                if cost < best_cost:
                    best_cost = cost
                    best_alternative = alt_frontier
        
        return best_alternative

class MultiRobotFrontierExplorer(Node):
    def __init__(self):
        super().__init__('multi_robot_frontier_explorer')
        
        self.declare_parameter('min_frontier_size', 0.2)
        self.declare_parameter('max_frontiers', 50)
        self.declare_parameter('distance_weight', 1.2)
        self.declare_parameter('size_weight', 1.0)
        self.declare_parameter('robot_namespaces', ['TB3_1', 'TB3_2'])
        self.declare_parameter('update_rate', 2.0)
        self.declare_parameter('debug_mode', False)
        
        self.robot_namespaces = self.get_parameter('robot_namespaces').value
        self.debug_mode = self.get_parameter('debug_mode').value
        update_rate = self.get_parameter('update_rate').value
        
        self.frontier_search = FastFrontierSearch(
            min_frontier_size=self.get_parameter('min_frontier_size').value,
            max_frontiers=self.get_parameter('max_frontiers').value
        )
        
        self.allocator = OptimalFrontierAllocator(
            distance_weight=self.get_parameter('distance_weight').value,
            size_weight=self.get_parameter('size_weight').value
        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        costmap_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/TB3_1/global_costmap/costmap', 
            self.costmap_callback,
            costmap_qos
        )
        
        self.marker_pub = self.create_publisher(MarkerArray, '/frontier_markers', 10)
        self.goal_pubs = {
            ns: self.create_publisher(PoseStamped, f'/{ns}/goal_pose', 10)
            for ns in self.robot_namespaces
        }
        
        self.robot_positions = {ns: None for ns in self.robot_namespaces}
        self.robot_goals = {ns: None for ns in self.robot_namespaces}
        self.last_goal_time = {ns: self.get_clock().now() for ns in self.robot_namespaces}
        
        self.costmap_received = False
        self.tf_warnings_shown = {ns: False for ns in self.robot_namespaces}
        self.allocation_count = 0
        
        self.create_timer(1.0 / update_rate, self.exploration_cycle)
        
    
    def costmap_callback(self, msg: OccupancyGrid):
        stats = self.frontier_search.update_costmap(msg)
        if stats is not None:
            self.get_logger().info(f'📍 Première costmap reçue. Stats: {stats}')
            self.costmap_received = True
    
    def update_robot_positions(self) -> bool:
        costmap_frame = self.frontier_search.costmap_frame
        if not costmap_frame:
            return False
        
        all_updated = True
        for ns in self.robot_namespaces:
            possible_frames = [f'{ns}/base_footprint', f'{ns}/base_link', f'{ns}/odom']
            position_found = False
            
            for source_frame in possible_frames:
                try:
                    t = self.tf_buffer.lookup_transform(
                        costmap_frame, source_frame, rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    pos = Point(x=t.transform.translation.x, y=t.transform.translation.y, z=0.0)
                    self.robot_positions[ns] = pos
                    position_found = True
                    break
                except TransformException:
                    continue
            
            if not position_found:
                all_updated = False
                if not self.tf_warnings_shown[ns]:
                    self.get_logger().warn(f'⚠️ Impossible de localiser {ns} dans {costmap_frame}')
                    self.tf_warnings_shown[ns] = True
        return all_updated
    
    def get_robot_state(self, ns: str) -> str:
        if self.robot_goals[ns] is None or self.robot_positions[ns] is None:
            return 'IDLE'
        
        current_pos = self.robot_positions[ns]
        goal = self.robot_goals[ns]
        dist = np.sqrt((current_pos.x - goal.x)**2 + (current_pos.y - goal.y)**2)
        
        if dist < GOAL_TOLERANCE:
            return 'IDLE'
        else:
            return 'BUSY'

    def exploration_cycle(self):
        if not self.costmap_received:
            return
        
        if not self.update_robot_positions():
            return
            
        frontiers = self.frontier_search.find_frontiers_fast(debug=self.debug_mode)
        
        if not frontiers:
            active_robots = [ns for ns in self.robot_namespaces if self.get_robot_state(ns) == 'IDLE']
            if active_robots and self.allocation_count % 10 == 0:
                self.get_logger().info(f"⚠️ Robots {active_robots} libres mais aucune frontière trouvée.")
            return
        
        current_positions = {ns: pos for ns, pos in self.robot_positions.items() if pos is not None}
        if not current_positions:
            return

        allocation = self.allocator.allocate(frontiers, current_positions, self.robot_goals)
        self.allocation_count += 1
        
        current_time = self.get_clock().now()
        
        for robot_ns, frontier in allocation.items():
            if frontier is not None:
                should_send = True
                dist_moved = 100.0
                
                if self.robot_goals[robot_ns] is not None:
                    last_goal = self.robot_goals[robot_ns]
                    dist_moved = np.sqrt((frontier.goal_point.x - last_goal.x)**2 + 
                                         (frontier.goal_point.y - last_goal.y)**2)
                    
                    # Si la cible a bougé de moins de 10cm, on ne renvoie pas, sauf si...
                    if dist_moved < 0.1: 
                        should_send = False
                        
                        # ... sauf si ça fait plus de 5 secondes (pour réveiller le robot bloqué)
                        time_since_last = (current_time - self.last_goal_time[robot_ns]).nanoseconds / 1e9
                        if time_since_last > FORCE_RESEND_TIME:
                            should_send = True
                            if self.debug_mode:
                                self.get_logger().info(f"⏰ Réveil forcé pour {robot_ns}")

                if should_send:
                    self.send_goal(robot_ns, frontier.goal_point)
                    self.robot_goals[robot_ns] = frontier.goal_point 
                    self.last_goal_time[robot_ns] = current_time
                    
                    if dist_moved > 0.1: # Log seulement les vrais changements
                        self.get_logger().info(f'🟢 {robot_ns} -> ({frontier.goal_point.x:.1f}, {frontier.goal_point.y:.1f})')
        
        self.visualize_frontiers(frontiers, allocation)

    def send_goal(self, robot_ns: str, point: Point):
        msg = PoseStamped()
        msg.header.frame_id = self.frontier_search.costmap_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position = point
        msg.pose.orientation.w = 1.0
        self.goal_pubs[robot_ns].publish(msg)
    
    def visualize_frontiers(self, frontiers: List[Frontier], 
                           allocation: Dict[str, Optional[Frontier]]):
        marker_array = MarkerArray()
        assigned_ids = set(id(f) for f in allocation.values() if f is not None)
        
        for i, frontier in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = self.frontier_search.costmap_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = marker.scale.y = 0.05
            
            if id(frontier) in assigned_ids:
                 marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0 
            else:
                 marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0 
            marker.color.a = 0.7
            marker.points = frontier.points
            marker_array.markers.append(marker)
            
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'labels'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position = frontier.centroid
            text_marker.pose.position.z = 0.5
            text_marker.scale.z = 0.2
            text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f'{frontier.area_m2:.1f}m'
            marker_array.markers.append(text_marker)
        
        for idx, (ns, goal) in enumerate(self.robot_goals.items()):
            if goal is not None:
                goal_marker = Marker()
                goal_marker.header.frame_id = self.frontier_search.costmap_frame
                goal_marker.header.stamp = self.get_clock().now().to_msg()
                goal_marker.ns = 'current_goals'
                goal_marker.id = idx + 2000
                goal_marker.type = Marker.SPHERE
                goal_marker.action = Marker.ADD
                goal_marker.pose.position = goal
                goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.3
                
                if '1' in ns: 
                    goal_marker.color.r = 0.0; goal_marker.color.g = 0.0; goal_marker.color.b = 1.0; goal_marker.color.a = 0.8
                else: 
                    goal_marker.color.r = 0.0; goal_marker.color.g = 1.0; goal_marker.color.b = 1.0; goal_marker.color.a = 0.8
                marker_array.markers.append(goal_marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotFrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
