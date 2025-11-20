import numpy as np
import cv2
import os
from typing import List, Tuple

class DetectionPointsInteretsUltraUltra:
    """Détection ultra-optimisée des points d'intérêt + visualisation avec robots et trajets."""

    def __init__(self, path: str, top_k: int = 50, dossier_debug: str = "debug"):
        self.top_k = top_k
        self.dossier_debug = dossier_debug
        self.valeur_pixel_supprime = -999999

        # Créer dossier debug
        os.makedirs(dossier_debug, exist_ok=True)

        # Étapes principales
        self.img = self.lecture_image(path)
        self.metriques_physiques()
        img_travail = self.transformer_valeurs(self.img)
        self.save(img_travail, "02_image_transformee.png")
        img_nettoyee = self.suppression_pixels_inutilisables(img_travail)
        self.heatmap_normalisee = self.lissage_normalisation(img_nettoyee)
        self.save(self.heatmap_normalisee, "07_heatmap_normalisee.png")

        # Superposition heatmap
        self.enregistrer_heatmap_superposee()

        # Top-K points d’intérêt
        self.points_interet = self.topk_points_interet(self.heatmap_normalisee, top_k=self.top_k)

        # Heatmap overlay avec points
        self.enregistrer_heatmap_overlay_avec_points()

    # ---------- Utilitaires ----------
    def save(self, img, filename):
        """Enregistre une image float32 en PNG."""
        out = img.copy()
        out[out <= self.valeur_pixel_supprime] = 0.0
        out = (out - out.min()) / (out.max() - out.min() + 1e-9)
        out8 = (out * 255).astype(np.uint8)
        cv2.imwrite(os.path.join(self.dossier_debug, filename), out8)

    # ---------- Étape 1 ----------
    def lecture_image(self, path: str) -> np.ndarray:
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Impossible de lire l'image : {path}")
        self.hauteur, self.largeur = img.shape
        self.save(img, "01_image_originale.png")
        return img

    # ---------- Étape 2 ----------
    def metriques_physiques(self):
        taille_pixel = 0.1
        taille_robot = 0.3
        h_px = max(1, int(taille_robot / taille_pixel))
        w_px = h_px
        self.matrice_robot = np.ones((h_px, w_px), dtype=np.float32)

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
        self.save(img_out, "03_suppression_autour_obstacles.png")

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

    # ---------- Étape 5 ----------
    def lissage_normalisation(self, img: np.ndarray) -> np.ndarray:
        mask_suppr = img <= self.valeur_pixel_supprime
        img2 = img.copy()

        # 1er passage
        tmp = img2.copy()
        tmp[mask_suppr] = 0
        img2[...] = cv2.boxFilter(tmp, -1, (3, 3), normalize=True)
        img2[mask_suppr] = self.valeur_pixel_supprime
        self.save(img2, "05_lissage_pass1.png")

        # 2e passage
        tmp = img2.copy()
        tmp[mask_suppr] = 0
        img2[...] = cv2.boxFilter(tmp, -1, (3, 3), normalize=True)
        img2[mask_suppr] = self.valeur_pixel_supprime
        self.save(img2, "06_lissage_pass2.png")

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
        """Retourne 1 point par zone connectée, tri par score."""
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

    # ---------- Visualisation ----------
    def enregistrer_heatmap_superposee(self, alpha=0.55, colormap=cv2.COLORMAP_TURBO):
        img_orig_color = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
        hm = np.clip(self.heatmap_normalisee, 0, 1)
        hm8 = (hm*255).astype(np.uint8)
        hm_color = cv2.applyColorMap(hm8, colormap)
        overlay = cv2.addWeighted(hm_color, alpha, img_orig_color, 1-alpha, 0)
        cv2.imwrite(os.path.join(self.dossier_debug,"heatmap_overlay.png"), overlay)
        print("[OK] Heatmap superposée enregistrée.")

    def enregistrer_heatmap_overlay_avec_points(self, alpha=0.55, colormap=cv2.COLORMAP_TURBO):
        """Superposition heatmap + points d'intérêt."""
        img_orig_color = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
        hm = np.clip(self.heatmap_normalisee, 0, 1)
        hm8 = (hm*255).astype(np.uint8)
        hm_color = cv2.applyColorMap(hm8, colormap)
        overlay = cv2.addWeighted(hm_color, alpha, img_orig_color, 1-alpha, 0)

        for (x, y, _) in self.points_interet:
            cv2.drawMarker(overlay, (x, y), (255,255,255),
                           markerType=cv2.MARKER_TILTED_CROSS, markerSize=8, thickness=2)
            cv2.drawMarker(overlay, (x, y), (0,0,255),
                           markerType=cv2.MARKER_TILTED_CROSS, markerSize=4, thickness=1)

        cv2.imwrite(os.path.join(self.dossier_debug,"heatmap_overlay_points.png"), overlay)
        print("[OK] Overlay + points d'intérêt enregistrés.")

    def enregistrer_heatmap_overlay_points_robots(self, robots: List[Tuple[int,int]],
                                                  targets: List[Tuple[int,int]],
                                                  paths: List[List[Tuple[int,int]]],
                                                  alpha=0.55, colormap=cv2.COLORMAP_TURBO):
        """Heatmap + points d'intérêt + robots + trajets."""
        img_orig_color = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
        hm = np.clip(self.heatmap_normalisee, 0, 1)
        hm8 = (hm*255).astype(np.uint8)
        hm_color = cv2.applyColorMap(hm8, colormap)
        overlay = cv2.addWeighted(hm_color, alpha, img_orig_color, 1-alpha, 0)

        # Points d'intérêt
        for (x, y, _) in self.points_interet:
            cv2.drawMarker(overlay, (x, y), (255,255,255),
                           markerType=cv2.MARKER_TILTED_CROSS, markerSize=8, thickness=2)
            cv2.drawMarker(overlay, (x, y), (0,0,255),
                           markerType=cv2.MARKER_TILTED_CROSS, markerSize=4, thickness=1)

        # Robots
        for (rx, ry) in robots:
            cv2.circle(overlay, (rx, ry), 5, (255,0,0), -1, lineType=cv2.LINE_AA)

        # Trajectoires
        for path in paths:
            if len(path) < 2:
                continue
            for i in range(len(path)-1):
                cv2.line(overlay, path[i], path[i+1], (0,255,0), 2, lineType=cv2.LINE_AA)

        cv2.imwrite(os.path.join(self.dossier_debug,"heatmap_overlay_points_robots.png"), overlay)
        print("[OK] Overlay + points + robots + trajets enregistrés.")


# -------- Multi-robots simplifié --------
from scipy.optimize import linear_sum_assignment
import heapq

class MultiRobotUltraUltra:
    def __init__(self, heatmap: np.ndarray, robots: List[Tuple[int,int]], points_interet: List[Tuple[int,int,float]]):
        """
        :param heatmap: heatmap normalisée
        :param robots: liste de positions initiales des robots [(x, y), ...]
        :param points_interet: liste de points d'intérêt [(x, y, score), ...]
        """
        self.heatmap = heatmap
        self.robots = robots
        self.points_interet = points_interet
        self.valeur_obstacle = -999999  # valeur des obstacles sur la heatmap

    # ---------- Assignation unique ----------
    def assign_targets_unique(self) -> List[Tuple[int,int]]:
        if len(self.points_interet) == 0 or len(self.robots) == 0:
            return [None]*len(self.robots)

        cost_matrix = np.zeros((len(self.robots), len(self.points_interet)), dtype=np.float32)
        for i, (rx, ry) in enumerate(self.robots):
            for j, (px, py, _) in enumerate(self.points_interet):
                cost_matrix[i, j] = np.hypot(px - rx, py - ry)

        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        targets = [None]*len(self.robots)
        for r, c in zip(row_ind, col_ind):
            targets[r] = (self.points_interet[c][0], self.points_interet[c][1])
        return targets

    # ---------- A* pour génération de chemins ----------
    def astar(self, start: Tuple[int,int], goal: Tuple[int,int]) -> List[Tuple[int,int]]:
        h, w = self.heatmap.shape

        def heuristic(a,b):
            return np.hypot(b[0]-a[0], b[1]-a[1])

        open_set = [(0 + heuristic(start, goal), 0, start, [start])]
        visited = set()

        while open_set:
            f, g, current, path = heapq.heappop(open_set)
            if current in visited:
                continue
            visited.add(current)

            if current == goal:
                return path

            x, y = current
            for dx in [-1,0,1]:
                for dy in [-1,0,1]:
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = x+dx, y+dy
                    if 0 <= nx < w and 0 <= ny < h:
                        if self.heatmap[ny, nx] > self.valeur_obstacle:
                            cost = g + np.hypot(dx, dy)
                            heapq.heappush(open_set, (cost + heuristic((nx,ny), goal), cost, (nx, ny), path + [(nx, ny)]))
        return []  # aucun chemin trouvé

    # ---------- Génération des chemins ----------
    def generate_paths(self, targets: List[Tuple[int,int]]) -> List[List[Tuple[int,int]]]:
        paths = []
        for r, t in zip(self.robots, targets):
            if t is None:
                paths.append([])
            else:
                path = self.astar(r, t)
                paths.append(path)
        return paths

    # ---------- Assignation + chemins tout-en-un ----------
    def assign_and_generate_paths(self) -> Tuple[List[Tuple[int,int]], List[List[Tuple[int,int]]]]:
        targets = self.assign_targets_unique()
        paths = self.generate_paths(targets)
        return targets, paths



# ---------------- Exemple ----------------
if __name__ == "__main__":
    det = DetectionPointsInteretsUltraUltra("map.pgm", top_k=50)
    robots = [(67,60), (50,90), (95,70)]

    multi = MultiRobotUltraUltra(det.heatmap_normalisee, robots, det.points_interet)
    targets, paths = multi.assign_and_generate_paths()

    # Enregistrement final
    det.enregistrer_heatmap_overlay_points_robots(robots, targets, paths)

