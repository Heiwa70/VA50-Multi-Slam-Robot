import numpy as np
import cv2
from typing import List, Tuple

# ---------------- Détection ultra-optimisée ----------------
class DetectionPointsInteretsUltraUltra:
    """Détection ultra-optimisée des points d'intérêt."""

    def __init__(self, path: str, top_k: int = 50):
        self.top_k = top_k
        self.valeur_pixel_supprime = -999999
        img = self.lecture_image(path)
        self.metriques_physiques()
        img_travail = self.transformer_valeurs(img)
        img_nettoyee = self.suppression_pixels_inutilisables(img_travail)
        self.heatmap_normalisee = self.lissage_normalisation(img_nettoyee)
        self.points_interet = self.topk_points_interet(self.heatmap_normalisee, top_k=self.top_k)

    def lecture_image(self, path: str) -> np.ndarray:
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(f"Impossible de lire l'image : {path}")
        self.hauteur, self.largeur = img.shape
        return img

    def metriques_physiques(self):
        taille_pixel = 0.1
        taille_robot = 0.3
        h_px = max(1, int(taille_robot / taille_pixel))
        w_px = h_px
        self.matrice_robot = np.ones((h_px, w_px), dtype=np.float32)

    def transformer_valeurs(self, img: np.ndarray) -> np.ndarray:
        img_f = img.astype(np.float32)
        # Mapping rapide avec vectorisation
        self.pixel_occupe = 128
        self.pixel_connu = 255
        self.pixel_inconnu = 0
        img_f[img == self.pixel_occupe] = -100
        img_f[img == self.pixel_connu] = 0
        img_f[img == self.pixel_inconnu] = 100
        return img_f

    def suppression_pixels_inutilisables(self, img: np.ndarray) -> np.ndarray:
        """Supprime pixels autour obstacles et pixels inconnus isolés (in-place)."""
        img_out = img  # on travaille in-place
        # zones autour obstacles
        kernel = self.matrice_robot / np.prod(self.matrice_robot.shape)
        mask_occ = (img_out == -100).astype(np.float32)
        zones = cv2.filter2D(mask_occ, -1, kernel, borderType=cv2.BORDER_CONSTANT)
        img_out[zones != 0] = self.valeur_pixel_supprime

        # pixels inconnus isolés
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

    def lissage_normalisation(self, img: np.ndarray) -> np.ndarray:
        """Double filtrage moyenneur + normalisation in-place."""
        mask_suppr = img == self.valeur_pixel_supprime
        for _ in range(2):
            tmp = img.copy()
            tmp[mask_suppr] = 0
            img[...] = cv2.boxFilter(tmp, -1, (3,3), normalize=True)
            img[mask_suppr] = self.valeur_pixel_supprime

        valid = img[~mask_suppr]
        if valid.size == 0: return np.zeros_like(img)
        mn, mx = valid.min(), valid.max()
        if mx == mn:
            norm = np.zeros_like(img)
        else:
            norm = (img - mn)/(mx - mn)
        norm[mask_suppr] = self.valeur_pixel_supprime
        return norm

    def topk_points_interet(self, heatmap: np.ndarray, top_k: int) -> List[Tuple[int,int,float]]:
        """Top-K points via np.argpartition (O(n))."""
        mask = heatmap > 0
        ys, xs = np.nonzero(mask)
        scores = heatmap[ys, xs]
        if scores.size > top_k:
            idx = np.argpartition(-scores, top_k)[:top_k]
            xs, ys, scores = xs[idx], ys[idx], scores[idx]
        # Tri final pour top-K
        idx_sort = np.argsort(-scores)
        xs, ys, scores = xs[idx_sort], ys[idx_sort], scores[idx_sort]
        return list(zip(xs, ys, scores))


# ---------------- Multi-robots vectorisé ----------------
class MultiRobotUltraUltra:
    """Multi-robots ultra-optimisé avec propagation vectorisée."""

    def __init__(self, heatmap: np.ndarray, robots: List[Tuple[int,int]]):
        self.heatmap = heatmap
        self.robots = robots
        self.targets = [None]*len(robots)

    def choose_targets(self, radius: int = 5, iter_dist: int = 5):
        """Assignation rapide robot → top-K points via propagation approximative."""
        dist_map = np.full_like(self.heatmap, np.inf, dtype=np.float32)
        mask = self.heatmap > 0
        dist_map[mask] = 0

        # Propagation 4-connectivité vectorisée
        for _ in range(iter_dist):
            dist_map[1:-1,1:-1] = np.minimum.reduce([
                dist_map[1:-1,1:-1],
                dist_map[:-2,1:-1]+1,
                dist_map[2:,1:-1]+1,
                dist_map[1:-1,:-2]+1,
                dist_map[1:-1,2:]+1
            ])

        for i, (rx, ry) in enumerate(self.robots):
            x0, x1 = max(0, rx-radius), min(self.heatmap.shape[1], rx+radius+1)
            y0, y1 = max(0, ry-radius), min(self.heatmap.shape[0], ry+radius+1)
            local_score = self.heatmap[y0:y1, x0:x1]
            if local_score.size == 0: continue
            max_idx = np.unravel_index(np.argmax(local_score), local_score.shape)
            self.targets[i] = (max_idx[1]+x0, max_idx[0]+y0)
        return self.targets


# ---------------- Exemple d'utilisation ----------------
if __name__ == "__main__":
    path = "mon_image.png"
    d = DetectionPointsInteretsUltraUltra(path, top_k=50)
    robots = [(10,10),(50,50),(100,20)]
    multi = MultiRobotUltraUltra(d.heatmap_normalisee, robots)
    targets = multi.choose_targets()
    print("Points d'intérêt choisis :", targets)
