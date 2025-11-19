import numpy as np
import cv2
import heapq
from typing import List, Tuple, Dict, Optional

# ------------------------
# Classe détection points d'intérêt
# ------------------------
class DetectionPointsInterets:
    def __init__(self, path: str):
        image_originelle = self.lecture_de_image(path)
        self.valeur_pixel_supprime = -999999
        self.metriques_physiques()
        image_travail = self.changement_des_valeurs_de_l_image(image_originelle)
        image_nettoyee = self.suppression_des_pixels_inutilisables_et_recherche_des_zones(image_travail)
        self.image_des_zones_a_explorer_moyennee_normalisee = self.traitement_de_image_nettoyee(image_nettoyee)
        self.points_interet = self.extraire_points_interet(
            self.image_des_zones_a_explorer_moyennee_normalisee, seuil_min=0.3, max_points=100
        )

    def lecture_de_image(self, path: str) -> np.ndarray:
        image_originelle = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if image_originelle is None:
            raise FileNotFoundError(f"Impossible de lire l'image : {path}")
        self.dimensions_image = {"hauteur": image_originelle.shape[0], "largeur": image_originelle.shape[1]}
        return image_originelle

    def metriques_physiques(self):
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
        masque_occupes = (img == self.valeur_pixel_occupe["image_voulue"]).astype(np.float32)
        image_filtre = cv2.filter2D(masque_occupes, ddepth=-1, kernel=filtre, borderType=cv2.BORDER_CONSTANT)
        img[image_filtre != 0] = self.valeur_pixel_supprime
        return img

    def suppression_des_pixels_inconnus_non_accessibles(self, image: np.ndarray) -> np.ndarray:
        img = image.copy()
        filtre_compt = np.array([
            [0.5,0.5,0.5,0.5,0.5],
            [0.5,1,1,1,0.5],
            [0.5,1,0,1,0.5],
            [0.5,1,1,1,0.5],
            [0.5,0.5,0.5,0.5,0.5]
        ], dtype=np.float32)
        masque_connus = (img == self.valeur_pixel_connu["image_voulue"]).astype(np.float32)
        voisins = cv2.filter2D(masque_connus, ddepth=-1, kernel=filtre_compt, borderType=cv2.BORDER_CONSTANT)
        masque_ok = (voisins > 1.0)
        img[~masque_ok] = self.valeur_pixel_supprime
        img[masque_ok] *= voisins[masque_ok]
        return img

    def application_du_filtre_moyenneur(self, image: np.ndarray) -> np.ndarray:
        img = image.copy()
        mask_suppr = (img == self.valeur_pixel_supprime)
        tmp = img.copy()
        tmp[mask_suppr] = 0.0
        filtre = np.ones((3,3), dtype=np.float32)/9
        img_moy = cv2.filter2D(tmp, ddepth=-1, kernel=filtre, borderType=cv2.BORDER_CONSTANT)
        img_moy[mask_suppr] = self.valeur_pixel_supprime
        return img_moy

    def traitement_de_image_nettoyee(self, image: np.ndarray) -> np.ndarray:
        img_moy1 = self.application_du_filtre_moyenneur(image)
        img_moy2 = self.application_du_filtre_moyenneur(img_moy1)
        return self.normalisation_de_image(img_moy2)

    def normalisation_de_image(self, image: np.ndarray) -> np.ndarray:
        img = image.copy()
        mask_suppr = (img == self.valeur_pixel_supprime)
        val_valides = img[~mask_suppr]
        if val_valides.size == 0: return np.zeros_like(img, dtype=np.float32)
        min_val, max_val = val_valides.min(), val_valides.max()
        if np.isclose(min_val, max_val):
            out = np.zeros_like(img, dtype=np.float32)
            out[mask_suppr] = self.valeur_pixel_supprime
            return out
        norm = (img - min_val) / (max_val - min_val)
        norm[mask_suppr] = self.valeur_pixel_supprime
        return norm

    def identification_des_zones_aux_pixels_connus_accessibles(self, image: np.ndarray) -> Tuple[Dict, np.ndarray]:
        image_bin = (image == self.valeur_pixel_connu["image_voulue"]).astype(np.uint8) * 255
        nb, labels, stats, _ = cv2.connectedComponentsWithStats(image_bin, connectivity=8)
        zones = {}
        for label in range(1, nb):
            top, left, h, w = stats[label, cv2.CC_STAT_TOP], stats[label, cv2.CC_STAT_LEFT], stats[label, cv2.CC_STAT_HEIGHT], stats[label, cv2.CC_STAT_WIDTH]
            mask_local = (labels[top:top+h, left:left+w] == label).astype(np.uint8)
            zones[label] = {"mask": mask_local, "offset": (left, top)}
        return zones, labels.astype(np.int32)

    def extraire_points_interet(self, heatmap: np.ndarray, seuil_min=0.1, max_points=None) -> List[Tuple[int,int,float]]:
        mask = heatmap >= seuil_min
        if not np.any(mask): return []
        ys, xs = np.where(mask)
        scores = heatmap[ys, xs]
        points = [(int(x), int(y), float(s)) for x,y,s in zip(xs, ys, scores)]
        points.sort(key=lambda t: t[2], reverse=True)
        if max_points: points = points[:max_points]
        return points

# ------------------------
# Classe Multi-robot optimisée avec A* inversé
# ------------------------
class MultiRobotExploration:
    def __init__(self, heatmap: np.ndarray, zones_connues: Dict, id_zones_par_pixel: np.ndarray, robot_positions: List[Tuple[int,int]]):
        self.heatmap = heatmap
        self.zones_connues = zones_connues
        self.id_zones_par_pixel = id_zones_par_pixel
        self.robot_positions = robot_positions
        self.robot_targets = [None]*len(robot_positions)

    def dijkstra_multi_cibles(self, occupancy_grid: np.ndarray, targets: List[Tuple[int,int]]) -> np.ndarray:
        """Retourne une carte distance depuis tous les targets vers chaque cellule libre (multi-cibles)."""
        h, w = occupancy_grid.shape
        dist_map = np.full((h,w), np.inf, dtype=np.float32)
        open_set = []
        for tx, ty in targets:
            if 0 <= tx < w and 0 <= ty < h and occupancy_grid[ty,tx] == 0:
                dist_map[ty, tx] = 0
                heapq.heappush(open_set, (0, (tx,ty)))
        neighbors = [(-1,0),(1,0),(0,-1),(0,1)]
        while open_set:
            d, (x,y) = heapq.heappop(open_set)
            if dist_map[y,x] < d: continue
            for dx,dy in neighbors:
                nx, ny = x+dx, y+dy
                if 0<=nx<w and 0<=ny<h and occupancy_grid[ny,nx]==0:
                    nd = d+1
                    if nd < dist_map[ny,nx]:
                        dist_map[ny,nx]=nd
                        heapq.heappush(open_set,(nd,(nx,ny)))
        return dist_map

    def choose_targets(self, occupancy_grid: np.ndarray) -> List[Optional[Tuple[int,int]]]:
        targets_all = [(x,y) for x,y,_ in np.ndenumerate(self.heatmap) if self.heatmap[y,x]>0]
        dist_map = self.dijkstra_multi_cibles(occupancy_grid, targets_all)
        chosen_targets = []
        for rx, ry in self.robot_positions:
            min_dist = np.inf
            best = None
            for x,y,_ in targets_all:
                if dist_map[ry,rx] < min_dist:
                    min_dist = dist_map[ry,rx]
                    best = (x,y)
            chosen_targets.append(best)
        self.robot_targets = chosen_targets
        return chosen_targets

# ------------------------
# Exemple d'utilisation
# ------------------------
if __name__ == "__main__":
    path_image = "mon_image.png"
    detection = DetectionPointsInterets(path_image)
    robots = [(10,10),(50,50),(100,20)]
    occupancy_grid = np.zeros_like(detection.id_des_zones_pour_chaque_pixel, dtype=np.uint8)
    multi_robot = MultiRobotExploration(
        detection.image_des_zones_a_explorer_moyennee_normalisee,
        detection.zones_connues,
        detection.id_des_zones_pour_chaque_pixel,
        robots
    )
    targets = multi_robot.choose_targets(occupancy_grid)
    print("Points d'intérêt choisis :", targets)
