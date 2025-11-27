import numpy as np
import cv2
import os
from typing import List, Tuple
from scipy.optimize import linear_sum_assignment
from skimage.graph import route_through_array

# ---------------- Détection des points d'intérêt ----------------
class DetectionPointsInterets:
    """Détection ultra-optimisée des points d'intérêt + visualisation avec robots et trajets."""

    def __init__(self, path: str, top_k: int = 50, dossier_debug: str = "debug"):
        self.top_k = top_k
        self.dossier_debug = dossier_debug
        self.valeur_pixel_supprime = -999999
        os.makedirs(dossier_debug, exist_ok=True)

        # Pipeline principal
        self.img = self.lecture_image(path)
        self.metriques_physiques()
        img_travail = self.transformer_valeurs(self.img)
        self.save(img_travail, "02_image_transformee.png")
        img_nettoyee = self.suppression_pixels_inutilisables(img_travail)
        self.heatmap_normalisee = self.lissage_normalisation(img_nettoyee)
        self.save(self.heatmap_normalisee, "07_heatmap_normalisee.png")

        # Génération des images
        self.enregistrer_heatmap_superposee()
        self.points_interet = self.topk_points_interet(self.heatmap_normalisee, top_k=self.top_k)
        self.enregistrer_heatmap_overlay_avec_points()

    # ---------- Utilitaires ----------
    def save(self, img, filename):
        """Enregistre une image float32 en PNG."""
        out = img.copy()
        out[out <= self.valeur_pixel_supprime] = 0.0
        out = (out - out.min()) / (out.max() - out.min() + 1e-9)
        out8 = (out * 255).astype(np.uint8)
        cv2.imwrite(os.path.join(self.dossier_debug, filename), out8)

    # ---------- Lecture ----------
    def lecture_image(self, path: str) -> np.ndarray:
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Impossible de lire l'image : {path}")
        self.hauteur, self.largeur = img.shape
        self.save(img, "01_image_originale.png")
        return img

    # ---------- Metrics physiques ----------
    def metriques_physiques(self):
        taille_pixel = 0.1
        taille_robot = 0.3
        h_px = max(1, int(taille_robot / taille_pixel))
        self.matrice_robot = np.ones((h_px, h_px), dtype=np.float32)

    # ---------- Transformation des valeurs ----------
    def transformer_valeurs(self, img: np.ndarray) -> np.ndarray:
        img_f = img.astype(np.float32)
        self.pixel_occupe = 0
        self.pixel_connu = 254
        self.pixel_inconnu = 205
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
        self.save(img_out, "04_inconnus_filtrage_voisinage.png")
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

    # ---------- Visualisation ----------
    def enregistrer_heatmap_superposee(self, alpha=0.55, colormap=cv2.COLORMAP_TURBO):
        img_orig_color = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
        hm8 = (np.clip(self.heatmap_normalisee,0,1)*255).astype(np.uint8)
        overlay = cv2.addWeighted(cv2.applyColorMap(hm8, colormap), alpha, img_orig_color, 1-alpha, 0)
        cv2.imwrite(os.path.join(self.dossier_debug,"heatmap_overlay.png"), overlay)

    def enregistrer_heatmap_overlay_avec_points(self, alpha=0.55, colormap=cv2.COLORMAP_TURBO):
        img_orig_color = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
        hm8 = (np.clip(self.heatmap_normalisee,0,1)*255).astype(np.uint8)
        overlay = cv2.addWeighted(cv2.applyColorMap(hm8, colormap), alpha, img_orig_color, 1-alpha, 0)
        for (x, y, _, _) in self.points_interet:
            cv2.drawMarker(overlay, (x, y), (0,0,255), markerType=cv2.MARKER_TILTED_CROSS, markerSize=6, thickness=1)
        cv2.imwrite(os.path.join(self.dossier_debug,"heatmap_overlay_points.png"), overlay)

    def enregistrer_heatmap_overlay_points_robots(self, robots: List[Tuple[int,int]], targets: List[Tuple[int,int]], paths: List[List[Tuple[int,int]]], alpha=0.55, colormap=cv2.COLORMAP_TURBO):
        img_orig_color = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
        hm8 = (np.clip(self.heatmap_normalisee,0,1)*255).astype(np.uint8)
        overlay = cv2.addWeighted(cv2.applyColorMap(hm8, colormap), alpha, img_orig_color, 1-alpha, 0)
        # Points
        for (x, y, _, _) in self.points_interet:
            cv2.drawMarker(overlay, (x, y), (0,0,255), markerType=cv2.MARKER_TILTED_CROSS, markerSize=6, thickness=1)
        # Robots
        for (rx, ry) in robots:
            cv2.circle(overlay, (rx, ry), 5, (255,0,0), -1)
        # Trajectoires
        for path in paths:
            for i in range(len(path)-1):
                cv2.line(overlay, path[i], path[i+1], (0,255,0), 2)
        cv2.imwrite(os.path.join(self.dossier_debug,"heatmap_overlay_points_robots.png"), overlay)


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


# ---------------- Exemple d'utilisation ----------------
if __name__ == "__main__":
    import time
    a = time.time()
    det = DetectionPointsInterets("map.pgm", top_k=50)
    robots = [(25,35), (30,25)]
    multi = MultiRobot(det.heatmap_normalisee, robots, det.points_interet)
    targets, paths = multi.assign_targets_optimized()
    det.enregistrer_heatmap_overlay_points_robots(robots, targets, paths)
    print("[INFO] Temps total :", time.time()-a)
