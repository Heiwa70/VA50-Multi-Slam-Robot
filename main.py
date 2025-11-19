import numpy as np
import cv2
import heapq
from typing import List, Tuple, Dict, Optional

class DetectionPointsInterets:
    """Classe optimisée pour la détection de points d'intérêt dans une carte d'exploration."""

    def __init__(self, path: str):
        image_originelle = self.lecture_de_image(path)
        
        self.valeur_pixel_supprime = -999999
        self.metriques_physiques()
        
        image_travail = self.changement_des_valeurs_de_l_image(image_originelle)
        image_nettoyee = self.suppression_des_pixels_inutilisables_et_recherche_des_zones(image_travail)
        self.image_des_zones_a_explorer_moyennee_normalisee = self.traitement_de_image_nettoyee(image_nettoyee)
        
        self.points_interet = self.extraire_points_interet(
            self.image_des_zones_a_explorer_moyennee_normalisee,
            seuil_min=0.3,
            max_points=100
        )

    def lecture_de_image(self, path: str) -> np.ndarray:
        image_originelle = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if image_originelle is None:
            raise FileNotFoundError(f"Impossible de lire l'image : {path}")
        self.dimensions_image = {"hauteur": image_originelle.shape[0], "largeur": image_originelle.shape[1]}
        return image_originelle

    def metriques_physiques(self) -> None:
        taille_pixel = {"hauteur": 0.1, "largeur": 0.1}
        taille_robot = {"hauteur": 0.3, "largeur": 0.3}
        self.taille_robot_en_pixel = {
            "hauteur": max(1, int(taille_robot["hauteur"] / taille_pixel["hauteur"])),
            "largeur": max(1, int(taille_robot["largeur"] / taille_pixel["largeur"]))
        }
        self.matrice_zone_robot = np.ones(
            (self.taille_robot_en_pixel["hauteur"], self.taille_robot_en_pixel["largeur"]), dtype=np.float32
        )

    def changement_des_valeurs_de_l_image(self, image: np.ndarray) -> np.ndarray:
        img = image.astype(np.float32)
        self.valeur_pixel_occupe = {"image_origine": 128, "image_voulue": -100}
        self.valeur_pixel_connu = {"image_origine": 255, "image_voulue": 0}
        self.valeur_pixel_inconnu = {"image_origine": 0, "image_voulue": 100}
        img[image == self.valeur_pixel_occupe["image_origine"]] = self.valeur_pixel_occupe["image_voulue"]
        img[image == self.valeur_pixel_connu["image_origine"]] = self.valeur_pixel_connu["image_voulue"]
        img[image == self.valeur_pixel_inconnu["image_origine"]] = self.valeur_pixel_inconnu["image_voulue"]
        return img

    def suppression_des_pixels_inutilisables_et_recherche_des_zones(self, image: np.ndarray) -> np.ndarray:
        image_sans_obstacles = self.suppression_des_zones_autour_des_obstacles(image)
        image_finale = self.suppression_des_pixels_inconnus_non_accessibles(image_sans_obstacles)
        self.zones_connues, self.id_des_zones_pour_chaque_pixel = self.identification_des_zones_aux_pixels_connus_accessibles(image_sans_obstacles)
        return image_finale

    def suppression_des_zones_autour_des_obstacles(self, image: np.ndarray) -> np.ndarray:
        img = image.copy()
        filtre = self.matrice_zone_robot / (self.taille_robot_en_pixel["hauteur"] * self.taille_robot_en_pixel["largeur"])
        masque = (img == self.valeur_pixel_occupe["image_voulue"]).astype(np.float32)
        zones_obstacles = cv2.filter2D(masque, ddepth=-1, kernel=filtre, borderType=cv2.BORDER_CONSTANT)
        img[zones_obstacles != 0] = self.valeur_pixel_supprime
        return img

    def suppression_des_pixels_inconnus_non_accessibles(self, image: np.ndarray) -> np.ndarray:
        img = image.copy()
        filtre = np.array([
            [0.5, 0.5, 0.5, 0.5, 0.5],
            [0.5, 1.0, 1.0, 1.0, 0.5],
            [0.5, 1.0, 0.0, 1.0, 0.5],
            [0.5, 1.0, 1.0, 1.0, 0.5],
            [0.5, 0.5, 0.5, 0.5, 0.5],
        ], dtype=np.float32)
        masque_connus = (img == self.valeur_pixel_connu["image_voulue"]).astype(np.float32)
        voisins = cv2.filter2D(masque_connus, ddepth=-1, kernel=filtre, borderType=cv2.BORDER_CONSTANT)
        conserve = voisins > 1.0
        img[~conserve] = self.valeur_pixel_supprime
        img[conserve] *= voisins[conserve]
        return img

    def application_du_filtre_moyenneur(self, image: np.ndarray) -> np.ndarray:
        img = image.copy()
        masque_sup = img == self.valeur_pixel_supprime
        tmp = img.copy()
        tmp[masque_sup] = 0.0
        filtre = np.ones((3,3), dtype=np.float32)/9.0
        result = cv2.filter2D(tmp, ddepth=-1, kernel=filtre, borderType=cv2.BORDER_CONSTANT)
        result[masque_sup] = self.valeur_pixel_supprime
        return result

    def traitement_de_image_nettoyee(self, image: np.ndarray) -> np.ndarray:
        img = self.application_du_filtre_moyenneur(image)
        img = self.application_du_filtre_moyenneur(img)
        return self.normalisation_de_image(img)

    def normalisation_de_image(self, image: np.ndarray) -> np.ndarray:
        img = image.copy()
        masque_sup = img == self.valeur_pixel_supprime
        valides = img[~masque_sup]
        if valides.size == 0: return np.zeros_like(img, dtype=np.float32)
        min_val, max_val = valides.min(), valides.max()
        if np.isclose(max_val, min_val):
            result = np.zeros_like(img)
            result[masque_sup] = self.valeur_pixel_supprime
            return result
        norm = (img - min_val) / (max_val - min_val)
        norm[masque_sup] = self.valeur_pixel_supprime
        return norm

    def identification_des_zones_aux_pixels_connus_accessibles(self, image: np.ndarray) -> Tuple[Dict, np.ndarray]:
        bin_img = (image == self.valeur_pixel_connu["image_voulue"]).astype(np.uint8)*255
        nb, labels, stats, _ = cv2.connectedComponentsWithStats(bin_img, connectivity=8)
        zones = {}
        for label in range(1, nb):
            top, left, h, w = stats[label, cv2.CC_STAT_TOP], stats[label, cv2.CC_STAT_LEFT], stats[label, cv2.CC_STAT_HEIGHT], stats[label, cv2.CC_STAT_WIDTH]
            mask = (labels[top:top+h, left:left+w] == label).astype(np.uint8)
            zones[label] = {"mask": mask, "offset": (left, top)}
        return zones, labels.astype(np.int32)

    def extraire_points_interet(self, heatmap: np.ndarray, seuil_min: float=0.1, max_points: Optional[int]=None) -> List[Tuple[int,int,float]]:
        mask = heatmap >= seuil_min
        if not np.any(mask): return []
        ys, xs = np.where(mask)
        scores = heatmap[ys, xs]
        points = [(int(x), int(y), float(s)) for x, y, s in zip(xs, ys, scores)]
        points.sort(key=lambda t: t[2], reverse=True)
        if max_points: points = points[:max_points]
        return points

    def recuperer_les_n_points_interets(self) -> List[Tuple[int,int,float]]:
        return self.points_interet

# ---------------------------------------------------
# Classe MultiRobotExploration optimisée
# ---------------------------------------------------
class MultiRobotExploration:
    def __init__(self, heatmap: np.ndarray, zones_connues: Dict, id_zones_par_pixel: np.ndarray, robot_positions: List[Tuple[int,int]]):
        self.heatmap = heatmap
        self.zones_connues = zones_connues
        self.id_zones_par_pixel = id_zones_par_pixel
        self.robot_positions = robot_positions
        self.robot_targets = [None]*len(robot_positions)

    def get_zone_of_robot(self, robot_idx: int) -> Optional[Dict]:
        x, y = self.robot_positions[robot_idx]
        h, w = self.id_zones_par_pixel.shape
        if not (0 <= x < w and 0 <= y < h): return None
        zone_id = int(self.id_zones_par_pixel[y, x])
        return self.zones_connues.get(zone_id, None)

    def get_points_of_interest(self, zone: Dict) -> List[Tuple[int,int,float]]:
        mask = zone["mask"]
        ox, oy = zone["offset"]
        ys, xs = np.where(mask)
        coords_x, coords_y = ox + xs, oy + ys
        scores = self.heatmap[coords_y, coords_x]
        points = [(int(x), int(y), float(s)) for x, y, s in zip(coords_x, coords_y, scores) if s>0]
        points.sort(key=lambda t: t[2], reverse=True)
        return points

    def astar(self, start: Tuple[int,int], goal: Tuple[int,int], occupancy_grid: np.ndarray) -> Optional[List[Tuple[int,int]]]:
        neighbors = [(-1,0),(1,0),(0,-1),(0,1)]
        h, w = occupancy_grid.shape
        open_set = []
        heapq.heappush(open_set, (self.heuristic(start,goal), 0, start, [start]))
        visited = set()
        while open_set:
            f, g, current, path = heapq.heappop(open_set)
            if current in visited: continue
            visited.add(current)
            if current == goal: return path
            for dx, dy in neighbors:
                nx, ny = current[0]+dx, current[1]+dy
                if 0<=nx<w and 0<=ny<h and occupancy_grid[ny,nx]==0 and (nx,ny) not in visited:
                    new_g = g+1
                    new_f = new_g + self.heuristic((nx,ny),goal)
                    heapq.heappush(open_set,(new_f,new_g,(nx,ny),path+[(nx,ny)]))
        return None

    def heuristic(self,a: Tuple[int,int], b: Tuple[int,int]) -> int:
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    def choose_targets(self, occupancy_grid: np.ndarray) -> List[Optional[Tuple[int,int]]]:
        """Attribue intelligemment les points d'intérêt aux robots pour éviter conflits."""
        assigned_points = set()
        targets = []
        for i, pos in enumerate(self.robot_positions):
            zone = self.get_zone_of_robot(i)
            if zone is None: 
                targets.append(None)
                continue
            points = self.get_points_of_interest(zone)
            best_point = None
            best_distance = float('inf')
            for px, py, score in points:
                if (px,py) in assigned_points: continue
                path = self.astar(pos,(px,py),occupancy_grid)
                if path is not None and len(path) < best_distance:
                    best_distance = len(path)
                    best_point = (px,py)
            if best_point: assigned_points.add(best_point)
            targets.append(best_point)
            self.robot_targets[i] = best_point
        return targets


if __name__ == "__main__":
    path_image = "mon_image.png"
    
    try:
        detection_des_points_d_interets = DetectionPointsInterets(path_image)
        
        # Positions initiales des robots
        robots = [(10, 10), (50, 50), (100, 20)]
        
        # Grille d'occupation : 0 = libre, 1 = obstacle
        occupancy_grid = np.zeros_like(
            detection_des_points_d_interets.id_des_zones_pour_chaque_pixel, 
            dtype=np.uint8
        )
        
        # Exemple : marquer une zone comme obstacle
        if 1 in detection_des_points_d_interets.zones_connues:
            zone = detection_des_points_d_interets.zones_connues[1]
            mask_global = np.zeros_like(occupancy_grid, dtype=np.uint8)
            ys, xs = np.where(zone["mask"])
            mask_global[zone["offset"][1] + ys, zone["offset"][0] + xs] = 1
            occupancy_grid = np.maximum(occupancy_grid, mask_global)
        
        # Planification multi-robots
        multi_robot = MultiRobotExploration(
            heatmap=detection_des_points_d_interets.image_des_zones_a_explorer_moyennee_normalisee,
            zones_connues=detection_des_points_d_interets.zones_connues,
            id_zones_par_pixel=detection_des_points_d_interets.id_des_zones_pour_chaque_pixel,
            robot_positions=robots
        )
        
        targets = multi_robot.choose_targets(occupancy_grid)
        print("Points d'intérêt choisis pour chaque robot :", targets)
        
    except FileNotFoundError as e:
        print(f"Erreur : {e}")
    except Exception as e:
        print(f"Erreur inattendue : {e}")