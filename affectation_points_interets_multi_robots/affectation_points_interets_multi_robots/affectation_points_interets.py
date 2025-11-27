import numpy as np
import cv2
import os
from typing import List, Tuple
from scipy.optimize import linear_sum_assignment
from skimage.graph import route_through_array

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import OccupancyGrid


# ---------------- Détection des points d'intérêt ----------------
class DetectionPointsInterets:

    def __init__(self, image, taille_pixel_en_metre, taille_robot_en_metre, top_k: int = 50):
        self.top_k = top_k
        self.valeur_pixel_supprime = -999999

        self.taille_pixel_en_metre = taille_pixel_en_metre
        self.taille_robot_en_metre = taille_robot_en_metre

        # Pipeline principal
        self.img = image
        self.metriques_physiques()
        img_travail = self.transformer_valeurs(self.img)
        img_nettoyee = self.suppression_pixels_inutilisables(img_travail)
        self.heatmap_normalisee = self.lissage_normalisation(img_nettoyee)

        self.points_interet = self.topk_points_interet(self.heatmap_normalisee, top_k=self.top_k)

    # ---------- Metrics physiques ----------
    def metriques_physiques(self):
        h_px = max(1, int(self.taille_robot_en_metre / self.taille_pixel_en_metre))
        self.matrice_robot = np.ones((h_px, h_px), dtype=np.float32)

    # ---------- Transformation des valeurs ----------
    def transformer_valeurs(self, img: np.ndarray) -> np.ndarray:
        img_f = img.astype(np.float32)
        self.pixel_occupe = 100
        self.pixel_connu = 0
        self.pixel_inconnu = -1
        img_f[img == self.pixel_occupe] = -100
        img_f[img == self.pixel_connu] = 0
        img_f[img == self.pixel_inconnu] = 100
        return img_f

    # ---------- Suppression des pixels inutiles ----------
    def suppression_pixels_inutilisables(self, img: np.ndarray) -> np.ndarray:
        img_out = img.copy()
        # Suppression autour des obstacles
        kernel = self.matrice_robot / np.prod(self.matrice_robot.shape)
        mask_occ = (img_out == -100).astype(np.float32)
        zones = cv2.filter2D(mask_occ, -1, kernel, borderType=cv2.BORDER_CONSTANT)
        img_out[zones != 0] = self.valeur_pixel_supprime

        # Filtrage des inconnus isolés
        mask_known = (img_out == 0).astype(np.float32)
        filtre_voisins = np.array([
            [0.5,0.5,0.5,0.5,0.5],
            [0.5,1.0,1.0,1.0,0.5],
            [0.5,1.0,0.0,1.0,0.5],
            [0.5,1.0,1.0,1.0,0.5],
            [0.5,0.5,0.5,0.5,0.5]
        ], dtype=np.float32)
        voisins = cv2.filter2D(mask_known, -1, filtre_voisins, borderType=cv2.BORDER_CONSTANT)
        mask_valid = voisins > 1.0
        img_out[~mask_valid] = self.valeur_pixel_supprime
        img_out[mask_valid] *= voisins[mask_valid]
        return img_out

    # ---------- Lissage et normalisation ----------
    def lissage_normalisation(self, img: np.ndarray) -> np.ndarray:
        mask_suppr = img <= self.valeur_pixel_supprime
        img2 = img.copy()

        # 2 passages de boxFilter
        for _ in range(2):
            tmp = img2.copy()
            tmp[mask_suppr] = 0
            img2[...] = cv2.boxFilter(tmp, -1, (3, 3), normalize=True)
            img2[mask_suppr] = self.valeur_pixel_supprime

        valid = img2[~mask_suppr]
        if valid.size == 0:
            return np.zeros_like(img2)
        mn, mx = valid.min(), valid.max()
        norm = np.zeros_like(img2) if mx == mn else (img2 - mn) / (mx - mn)
        norm[mask_suppr] = self.valeur_pixel_supprime
        return norm

    # ---------- Top-K points d'intérêt ----------
    def topk_points_interet(self, heatmap: np.ndarray, top_k: int) -> List[Tuple[int,int,float,int]]:
        mask = heatmap > 0
        num_labels, labels = cv2.connectedComponents(mask.astype(np.uint8))
        points = []

        for lab in range(1, num_labels):
            ys, xs = np.where(labels == lab)
            if len(xs) == 0:
                continue
            scores = heatmap[ys, xs]
            idx_best = np.argmax(scores)
            zone_size = len(xs)
            points.append((xs[idx_best], ys[idx_best], float(scores[idx_best]), zone_size))

        # Trier par taille puis score
        points.sort(key=lambda p: (-p[3], -p[2]))
        return points[:top_k]


# ---------------- Multi-Robot Optimisé ----------------
class MultiRobot:
    def __init__(self, heatmap: np.ndarray, robots: List[Tuple[int,int]], points_interet: List[Tuple[int,int,float,int]]):
        self.heatmap = heatmap
        self.robots = robots
        self.points_interet = points_interet
        self.valeur_obstacle = -999999
        self.cost_map = np.ones_like(self.heatmap, dtype=np.float32)
        self.cost_map[self.heatmap <= self.valeur_obstacle] = np.inf

    def assign_targets_optimized(self):
        n_r, n_p = len(self.robots), len(self.points_interet)
        if n_r == 0 or n_p == 0:
            return [None]*n_r, [[] for _ in range(n_r)]

        # Calcul des coûts initiaux
        cost_matrix = np.full((n_r, n_p), np.inf, dtype=np.float32)
        paths_matrix = [[[] for _ in range(n_p)] for _ in range(n_r)]
        for i, (rx, ry) in enumerate(self.robots):
            for j, (px, py, score, size) in enumerate(self.points_interet):
                try:
                    path, cost = route_through_array(self.cost_map, start=(ry, rx), end=(py, px), fully_connected=True)
                    cost_matrix[i, j] = cost / (1 + size)
                    paths_matrix[i][j] = [(c, r) for r, c in path]
                except:
                    continue

        # Assignation initiale via Hungarian
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        targets = [None]*n_r
        paths = [[] for _ in range(n_r)]
        assigned_points = []
        for r, c in zip(row_ind, col_ind):
            if cost_matrix[r, c] < np.inf:
                targets[r] = (self.points_interet[c][0], self.points_interet[c][1])
                paths[r] = paths_matrix[r][c]
                assigned_points.append(self.points_interet[c][:2])

        # Réajuster les points trop proches
        for i in range(n_r):
            for j, (px, py, score, size) in enumerate(self.points_interet):
                if (px, py) in assigned_points:
                    continue
                if not assigned_points:
                    continue
                dist = min(np.hypot(px - ax, py - ay) for (ax, ay) in assigned_points)
                cost_matrix[i, j] *= (1 + np.exp(-dist/5))  # pénalisation auto

        # Nouvelle assignation optimisée
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        targets = [None]*n_r
        paths = [[] for _ in range(n_r)]
        for r, c in zip(row_ind, col_ind):
            if cost_matrix[r, c] < np.inf:
                targets[r] = (self.points_interet[c][0], self.points_interet[c][1])
                paths[r] = paths_matrix[r][c]

        return targets, paths


class AffectationPointsInterets(Node):

    def __init__(self):
        super().__init__('affectation_points_interets')

        self.cv_bridge = CvBridge()

        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.scan_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos)
        self.scan_subscriber  
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.positions_robots = {}

        self.map_info = None
        self.map_received = False


        # Liste des robots à surveiller
        self.robot_frames = [
            "robot1/base_link",
            "robot2/base_link",
            "robot3/base_link"
        ]

        self.timer = self.create_timer(0.1, self.update_poses)

    def update_poses(self):
        if not self.map_received:
            return

        info = self.map_info
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        res = info.resolution

        for frame in self.robot_frames:
            try:
                tr = self.tf_buffer.lookup_transform(
                    "map", frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
                )

                x = tr.transform.translation.x
                y = tr.transform.translation.y

                q = tr.transform.rotation
                yaw = np.arctan2(
                    2*(q.w*q.z + q.x*q.y),
                    1 - 2*(q.y*q.y + q.z*q.z)
                )

                # Conversion en pixels
                px = int((x - origin_x) / res)
                py = int((y - origin_y) / res)

                print(f"Robot {frame}: px={px}, py={py}, yaw={yaw:.2f}")

                self.positions_robots[frame] = {
                    "x": x, "y": y, "px": px, "py": py, "yaw": yaw
                }

            except Exception:
                pass



    def map_callback(self, msg: OccupancyGrid):

        # Stocker les infos de la carte
        self.map_info = msg.info
        self.map_received = True

        if len(self.positions_robots) == 0:
            return

        width  = msg.info.width
        height = msg.info.height
        taille_pixel_en_metre = msg.info.resolution
        taille_robot_en_metre = 0.3

        self.get_logger().info(f"Carte reçue : {width} x {height} px")
        self.get_logger().info(f"Taille d'un pixel : {taille_pixel_en_metre:.2f} m")

        carte = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

        det = DetectionPointsInterets(carte, taille_pixel_en_metre, taille_robot_en_metre, top_k=50)

        robots = []
        nom_robot = {}

        for key, value in self.positions_robots.items():
            robots.append((value["px"], value["py"]))
            nom_robot[(value["px"], value["py"])] = key


        multi = MultiRobot(det.heatmap_normalisee, robots, det.points_interet)
        targets, paths = multi.assign_targets_optimized()

        for n in range(len(paths)):
            start = robots[n]          # position pixel du robot n
            robot_name = nom_robot[start]
            start_px, start_py = paths[n][0]
            end_px, end_py = paths[n][-1]

            print(f"Robot {robot_name} : départ=({start_px},{start_py}) → objectif=({end_px},{end_py})")




def main(args=None):
    rclpy.init(args=args)

    affectation_points_interets = AffectationPointsInterets()
    rclpy.spin(affectation_points_interets)

    affectation_points_interets.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


     
