import numpy as np
import cv2
import os
from typing import List, Tuple


class DetectionPointsInteretsUltraUltra:
    """Détection ultra-optimisée des points d'intérêt + enregistrement de chaque étape."""

    def __init__(self, path: str, top_k: int = 50, dossier_debug: str = "debug"):
        self.top_k = top_k
        self.dossier_debug = dossier_debug
        self.valeur_pixel_supprime = -999999

        # Créer dossier debug
        os.makedirs(dossier_debug, exist_ok=True)

        # Étape 1 : lecture
        self.img = self.lecture_image(path)

        # Étape 2 : métriques robot
        self.metriques_physiques()

        # Étape 3 : mapping valeurs
        img_travail = self.transformer_valeurs(self.img)
        self.save(img_travail, "02_image_transformee.png")

        # Étape 4 : nettoyage
        img_nettoyee = self.suppression_pixels_inutilisables(img_travail)

        # Étape 5 : lissage + normalisation finale
        self.heatmap_normalisee = self.lissage_normalisation(img_nettoyee)
        self.save(self.heatmap_normalisee, "07_heatmap_normalisee.png")


        self.enregistrer_heatmap_superposee()

        # Étape 6 : top-K
        self.points_interet = self.topk_points_interet(self.heatmap_normalisee, top_k=self.top_k)

        self.enregistrer_heatmap_overlay_avec_points()

    # ---------- Utilitaires ----------
    def save(self, img, filename):
        """Enregistre une image float32 en PNG avec conversion automatique."""
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
        img_out = img

        # 4.1 suppression autour obstacles
        kernel = self.matrice_robot / np.prod(self.matrice_robot.shape)
        mask_occ = (img_out == -100).astype(np.float32)
        zones = cv2.filter2D(mask_occ, -1, kernel, borderType=cv2.BORDER_CONSTANT)
        img_out[zones != 0] = self.valeur_pixel_supprime
        self.save(img_out, "03_suppression_autour_obstacles.png")

        # 4.2 suppression inconnus isolés
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

        # normalisation finale
        valid = img2[~mask_suppr]
        if valid.size == 0:
            return np.zeros_like(img2)
        mn, mx = valid.min(), valid.max()
        if mx == mn:
            norm = np.zeros_like(img2)
        else:
            norm = (img2 - mn) / (mx - mn)

        norm[mask_suppr] = self.valeur_pixel_supprime
        return norm

    # ---------- Étape 6 ----------
    def topk_points_interet(self, heatmap: np.ndarray, top_k: int) -> List[Tuple[int,int,float]]:
        """
        Retourne 1 point d'intérêt par zone :
        - segmentation connected components
        - pour chaque composante, on prend le pixel de score maximum
        - on trie les points par score
        - on applique top_k facultatif
        """
        # Mask des zones valides
        mask = heatmap > 0

        # 1️⃣ Segmentation en zones connectées
        num_labels, labels = cv2.connectedComponents(mask.astype(np.uint8))

        points = []

        # 2️⃣ Pour chaque zone (sauf 0 : fond)
        for lab in range(1, num_labels):
            ys, xs = np.where(labels == lab)
            if len(xs) == 0:
                continue

            # Scores de la zone
            scores = heatmap[ys, xs]

            # Best pixel of the zone
            idx_best = np.argmax(scores)
            best_x = xs[idx_best]
            best_y = ys[idx_best]
            best_score = scores[idx_best]

            points.append((best_x, best_y, float(best_score)))

        # 3️⃣ Tri décroissant
        points.sort(key=lambda p: -p[2])

        # 4️⃣ Optionnel : top-K
        if len(points) > top_k:
            points = points[:top_k]

        return points

    
    def enregistrer_heatmap_superposee(self, alpha=0.55, colormap=cv2.COLORMAP_TURBO):

        # ---- Charger l'image originale ----
        img_orig = self.img

        # Convertir en BGR pour superposition
        img_orig_color = cv2.cvtColor(img_orig, cv2.COLOR_GRAY2BGR)

        # ---- Préparer la heatmap ----
        hm = self.heatmap_normalisee.copy()
        hm[hm == 0.0] = 1
        hm[hm == self.valeur_pixel_supprime] = 0.0  # pixels invalides à noir

        # Convertir → 8 bits
        hm8 = (hm * 255).clip(0, 255).astype(np.uint8)

        # Appliquer la color map (BGR)
        hm_color = cv2.applyColorMap(hm8, colormap)

        # ---- Superposition ----
        overlay = cv2.addWeighted(hm_color, alpha, img_orig_color, 1 - alpha, 0)
        output = os.path.join(self.dossier_debug,"heatmap_overlay.png")
        # ---- Sauvegarde ----
        cv2.imwrite(output, overlay)
        print(f"[OK] Heatmap superposée enregistrée : {output}")



    def enregistrer_heatmap_overlay_avec_points(self,
                                                alpha=0.55, colormap=cv2.COLORMAP_TURBO):
        """
        Version améliorée : heatmap colorisée + superposition sur l'image +
        croix sur tous les points d'intérêt détectés.
        """

        output = "heatmap_overlay_points.png"

        # ---- Charger l'image originale ----
        img_orig = self.img
        img_orig_color = cv2.cvtColor(img_orig, cv2.COLOR_GRAY2BGR)

        # ---- Préparer la heatmap ----
        hm = self.heatmap_normalisee.copy()
        hm[hm == self.valeur_pixel_supprime] = 0.0
        hm8 = (hm * 255).clip(0, 255).astype(np.uint8)

        # ---- Colorisation ----
        hm_color = cv2.applyColorMap(hm8, colormap)

        # ---- Superposition ----
        overlay = cv2.addWeighted(hm_color, alpha, img_orig_color, 1 - alpha, 0)

        # ---- Dessiner les points d'intérêt ----
        for (x, y, score) in self.points_interet:
            cv2.drawMarker(
                overlay,
                (x, y),
                (0, 0, 0),                   # centre noir
                markerType=cv2.MARKER_CROSS,
                markerSize=12,
                thickness=2,
                line_type=cv2.LINE_AA
            )
            cv2.drawMarker(
                overlay,
                (x, y),
                (0, 255, 0),                 # contour vert
                markerType=cv2.MARKER_CROSS,
                markerSize=8,
                thickness=1,
                line_type=cv2.LINE_AA
            )

        filepath = os.path.join(self.dossier_debug, output)
        cv2.imwrite(filepath, overlay)
        print(f"[OK] Overlay + points d'intérêt enregistrés : {filepath}")

# -------- Multi-robots simplifié --------
class MultiRobotUltraUltra:
    def __init__(self, heatmap: np.ndarray, robots: List[Tuple[int,int]]):
        self.heatmap = heatmap
        self.robots = robots

    def choose_targets(self, radius: int = 5):
        targets = []
        for (rx, ry) in self.robots:
            x0, x1 = max(0, rx-radius), min(self.heatmap.shape[1], rx+radius+1)
            y0, y1 = max(0, ry-radius), min(self.heatmap.shape[0], ry+radius+1)
            local = self.heatmap[y0:y1, x0:x1]
            if local.size == 0:
                targets.append(None)
                continue
            y, x = np.unravel_index(np.argmax(local), local.shape)
            targets.append((x + x0, y + y0))
        return targets


# ---------------- Exemple ----------------
if __name__ == "__main__":
    det = DetectionPointsInteretsUltraUltra("map.pgm", top_k=50)
    robots = [(10,10), (50,50), (100,20)]
    multi = MultiRobotUltraUltra(det.heatmap_normalisee, robots)
    print("Cibles robots :", multi.choose_targets())
