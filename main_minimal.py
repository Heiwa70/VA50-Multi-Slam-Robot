import numpy as np
import cv2
from typing import List, Tuple
from scipy.optimize import linear_sum_assignment
from skimage.graph import route_through_array
import time

class DetectionPointsInteretsUltraUltra:
    """Détection ultra-optimisée des points d'intérêt."""

    def __init__(self, path: str, top_k: int = 50):
        self.top_k = top_k
        self.valeur_pixel_supprime = -999999

        # Étapes principales
        self.img = self.lecture_image(path)
        self.metriques_physiques()
        img_travail = self.transformer_valeurs(self.img)
        img_nettoyee = self.suppression_pixels_inutilisables(img_travail)
        self.heatmap_normalisee = self.lissage_normalisation(img_nettoyee)

        # Top-K points d’intérêt
        self.points_interet = self.topk_points_interet(self.heatmap_normalisee, top_k=self.top_k)

    # ---------- Étape 1 ----------
    def lecture_image(self, path: str) -> np.ndarray:
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Impossible de lire l'image : {path}")
        self.hauteur, self.largeur = img.shape
        return img

    # ---------- Étape 2 ----------
    def metriques_physiques(self):
        taille_pixel = 0.1
        taille_robot = 0.3
        h_px = max(1, int(taille_robot / taille_pixel))
        self.matrice_robot = np.ones((h_px, h_px), dtype=np.float32)

    # ---------- Étape 3 ----------
    def transformer_valeurs(self, img: np.ndarray) -> np.ndarray:
        img_f = img.astype(np.float32)
        self.pixel_occupe = 0
        self.pixel_connu = 254
        self.pixel_inconnu = 205
        img_f[img == self.pixel_occupe] = -100
        img_f[img == self.pixel_connu] = 0
        img_f[img == self.pixel_inconnu] = 100
        return img_f

    # ---------- Étape 4 ----------
    def suppression_pixels_inutilisables(self, img: np.ndarray) -> np.ndarray:
        img_out = img.copy()
        # Supprimer autour des obstacles
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

    # ---------- Étape 5 ----------
    def lissage_normalisation(self, img: np.ndarray) -> np.ndarray:
        mask_suppr = img <= self.valeur_pixel_supprime
        img2 = img.astype(np.float32, copy=False)

        # 1er passage
        tmp = img2.copy()
        tmp[mask_suppr] = 0
        cv2.boxFilter(tmp, -1, (3, 3), dst=img2, normalize=True)
        img2[mask_suppr] = self.valeur_pixel_supprime

        # 2e passage
        tmp = img2.copy()
        tmp[mask_suppr] = 0
        cv2.boxFilter(tmp, -1, (3, 3), dst=img2, normalize=True)
        img2[mask_suppr] = self.valeur_pixel_supprime

        # Normalisation finale
        valid = img2[~mask_suppr]
        if valid.size == 0:
            return np.zeros_like(img2)

        mn, mx = valid.min(), valid.max()
        norm = np.zeros_like(img2) if mx == mn else (img2 - mn) / (mx - mn)
        norm[mask_suppr] = self.valeur_pixel_supprime
        return norm

    # ---------- Étape 6 ----------
    def topk_points_interet(self, heatmap: np.ndarray, top_k: int) -> List[Tuple[int,int,float]]:
        mask = heatmap > 0
        num_labels, labels = cv2.connectedComponents(mask.astype(np.uint8))
        points = []

        for lab in range(1, num_labels):
            ys, xs = np.where(labels == lab)
            if len(xs) == 0:
                continue
            scores = heatmap[ys, xs]
            idx_best = np.argmax(scores)
            points.append((xs[idx_best], ys[idx_best], float(scores[idx_best])))

        points.sort(key=lambda p: -p[2])
        return points[:top_k] if len(points) > top_k else points


class MultiRobotUltraUltra:
    def __init__(self, heatmap: np.ndarray, robots: List[Tuple[int,int]], points_interet: List[Tuple[int,int,float]]):
        self.heatmap = heatmap
        self.robots = robots
        self.points_interet = points_interet
        self.valeur_obstacle = -999999
        self.cost_map = np.ones_like(self.heatmap, dtype=np.float32)
        self.cost_map[self.heatmap <= self.valeur_obstacle] = np.inf

    def assign_targets_fast(self) -> Tuple[List[Tuple[int,int]], List[List[Tuple[int,int]]]]:
        n_r, n_p = len(self.robots), len(self.points_interet)
        if n_r == 0 or n_p == 0:
            return [None]*n_r, [[] for _ in range(n_r)]

        cost_matrix = np.full((n_r, n_p), np.inf, dtype=np.float32)
        paths_matrix = [[[] for _ in range(n_p)] for _ in range(n_r)]

        for i, (rx, ry) in enumerate(self.robots):
            for j, (px, py, _) in enumerate(self.points_interet):
                try:
                    path, cost = route_through_array(
                        self.cost_map, start=(ry, rx), end=(py, px), fully_connected=True
                    )
                    cost_matrix[i, j] = cost
                    paths_matrix[i][j] = [(c, r) for r, c in path]
                except Exception:
                    cost_matrix[i, j] = np.inf
                    paths_matrix[i][j] = []

        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        targets = [None]*n_r
        paths = [[] for _ in range(n_r)]
        for r, c in zip(row_ind, col_ind):
            if cost_matrix[r, c] < np.inf:
                targets[r] = (self.points_interet[c][0], self.points_interet[c][1])
                paths[r] = paths_matrix[r][c]

        return targets, paths


# ---------------- Exemple ----------------
if __name__ == "__main__":
    start_time = time.time()
    det = DetectionPointsInteretsUltraUltra("map.pgm", top_k=50)
    robots = [(67, 60), (50, 90), (95, 70)]

    multi = MultiRobotUltraUltra(det.heatmap_normalisee, robots, det.points_interet)
    targets, paths = multi.assign_targets_fast()

    # Affichage clair des assignations
    print("Assignation des points d'intérêt aux robots :")
    for idx, (robot, target) in enumerate(zip(robots, targets)):
        print(f" Robot {idx} at {robot} → Point attribué : {target}")

    print("\nTemps total :", round(time.time() - start_time, 3), "secondes")
