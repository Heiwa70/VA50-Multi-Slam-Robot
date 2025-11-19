import numpy as np
import cv2
import heapq
from typing import List, Tuple, Dict, Optional

class DetectionPointsInteretsUltra:
    """Détection ultra-optimisée des points d'intérêt dans une carte d'exploration."""

    def __init__(self, path: str, top_k: int = 50):
        self.top_k = top_k
        image_originelle = self.lecture_de_image(path)
        self.valeur_pixel_supprime = -999999
        self.metriques_physiques()
        image_travail = self.changement_des_valeurs_de_l_image(image_originelle)
        image_nettoyee = self.suppression_pixels_inutilisables(image_travail)
        self.heatmap_normalisee = self.lissage_et_normalisation(image_nettoyee)
        self.points_interet = self.extraire_points_interet_topk(self.heatmap_normalisee, top_k=self.top_k)

    def lecture_de_image(self, path: str) -> np.ndarray:
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Impossible de lire l'image : {path}")
        self.hauteur, self.largeur = img.shape
        return img

    def metriques_physiques(self):
        # taille en pixels du robot
        taille_pixel = 0.1
        taille_robot = 0.3
        h_px = max(1, int(taille_robot / taille_pixel))
        w_px = max(1, int(taille_robot / taille_pixel))
        self.matrice_robot = np.ones((h_px, w_px), dtype=np.float32)

    def changement_des_valeurs_de_l_image(self, img: np.ndarray) -> np.ndarray:
        img_f = img.astype(np.float32)
        self.pixel_occupe = 128
        self.pixel_connu = 255
        self.pixel_inconnu = 0
        img_f[img == self.pixel_occupe] = -100
        img_f[img == self.pixel_connu] = 0
        img_f[img == self.pixel_inconnu] = 100
        return img_f

    def suppression_pixels_inutilisables(self, img: np.ndarray) -> np.ndarray:
        """Supprime pixels autour obstacles et pixels inconnus isolés."""
        img_out = img.copy()
        # zones autour obstacles
        kernel = self.matrice_robot / np.prod(self.matrice_robot.shape)
        mask_occ = (img_out == -100).astype(np.float32)
        zones = cv2.filter2D(mask_occ, -1, kernel, borderType=cv2.BORDER_CONSTANT)
        img_out[zones != 0] = self.valeur_pixel_supprime
        # pixels inconnus isolés
        mask_known = (img_out == 0).astype(np.float32)
        filtre_voisins = np.array([
            [0.5, 0.5, 0.5, 0.5, 0.5],
            [0.5, 1.0, 1.0, 1.0, 0.5],
            [0.5, 1.0, 0.0, 1.0, 0.5],
            [0.5, 1.0, 1.0, 1.0, 0.5],
            [0.5, 0.5, 0.5, 0.5, 0.5]
        ], dtype=np.float32)
        voisins = cv2.filter2D(mask_known, -1, filtre_voisins, borderType=cv2.BORDER_CONSTANT)
        img_out[voisins <= 1.0] = self.valeur_pixel_supprime
        img_out[voisins > 1.0] *= voisins[voisins > 1.0]
        return img_out

    def lissage_et_normalisation(self, img: np.ndarray) -> np.ndarray:
        """Double filtrage moyenneur + normalisation 0-1 excluant pixels supprimés."""
        img_tmp = img.copy()
        mask_suppr = img_tmp == self.valeur_pixel_supprime
        for _ in range(2):
            tmp = img_tmp.copy()
            tmp[mask_suppr] = 0.0
            img_tmp = cv2.filter2D(tmp, -1, np.ones((3,3), dtype=np.float32)/9, borderType=cv2.BORDER_CONSTANT)
            img_tmp[mask_suppr] = self.valeur_pixel_supprime
        mask = mask_suppr
        valid = img_tmp[~mask]
        if valid.size == 0: return np.zeros_like(img_tmp)
        mn, mx = valid.min(), valid.max()
        if mx == mn: norm = np.zeros_like(img_tmp)
        else: norm = (img_tmp - mn)/(mx - mn)
        norm[mask] = self.valeur_pixel_supprime
        return norm

    def extraire_points_interet_topk(self, heatmap: np.ndarray, top_k: int) -> List[Tuple[int,int,float]]:
        """Retourne top-K points d’intérêt vectorisés."""
        mask = heatmap > 0
        ys, xs = np.where(mask)
        scores = heatmap[ys, xs]
        pts = list(zip(xs, ys, scores))
        pts.sort(key=lambda t: t[2], reverse=True)
        return pts[:top_k]

# ---------------- Multi-robots ultra-optimisé ----------------

class MultiRobotUltra:
    """Planification ultra-optimisée multi-robots."""

    def __init__(self, heatmap: np.ndarray, robots: List[Tuple[int,int]]):
        self.heatmap = heatmap
        self.robots = robots
        self.targets = [None]*len(robots)

    def choose_targets(self):
        """Choix top-K par robot sans A* individuel."""
        # Dijkstra inversé depuis top-K points
        mask = self.heatmap > 0
        dist_map = np.full_like(self.heatmap, np.inf, dtype=np.float32)
        ys, xs = np.where(mask)
        for x, y in zip(xs, ys):
            dist_map[y, x] = 0
        # Simple propagation 4-connectivité (approximation pour gain de vitesse)
        for _ in range(5):  # 5 itérations suffisent pour une estimation
            dist_map[1:-1,1:-1] = np.minimum.reduce([
                dist_map[1:-1,1:-1],
                dist_map[:-2,1:-1]+1,
                dist_map[2:,1:-1]+1,
                dist_map[1:-1,:-2]+1,
                dist_map[1:-1,2:]+1
            ])
        # Assignation robot → meilleur point
        for i, (rx, ry) in enumerate(self.robots):
            local_score = self.heatmap[max(0,ry-5):ry+6, max(0,rx-5):rx+6]
            if local_score.size == 0: continue
            max_idx = np.unravel_index(np.argmax(local_score), local_score.shape)
            self.targets[i] = (max_idx[1]+max(0,rx-5), max_idx[0]+max(0,ry-5))
        return self.targets

# ---------------- Exemple d'utilisation ----------------

if __name__ == "__main__":
    path = "mon_image.png"
    d = DetectionPointsInteretsUltra(path, top_k=50)
    robots = [(10,10),(50,50),(100,20)]
    multi = MultiRobotUltra(d.heatmap_normalisee, robots)
    targets = multi.choose_targets()
    print("Points d'intérêt choisis :", targets)
