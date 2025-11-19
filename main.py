import numpy as np
import cv2
import heapq
from typing import List, Tuple, Dict, Optional

class DetectionPointsInterets:
    """Classe optimisée pour la détection de points d'intérêt dans une carte d'exploration."""

    def __init__(self, path: str):
        image_originelle = self.lecture_de_image(path)
        
        # Valeur sentinelle pour "pixel supprimé"
        self.valeur_pixel_supprime = -999999
        
        self.metriques_physiques()
        
        image_changement_de_valeur = self.changement_des_valeurs_de_l_image(image_originelle)
        
        image_nettoyee = self.suppression_des_pixels_inutilisables_et_recherche_des_zones(
            image_changement_de_valeur
        )
        
        self.image_des_zones_a_explorer_moyennee_normalisee = self.traitement_de_image_nettoyee(
            image_nettoyee
        )
        
        self.points_interet = self.extraire_points_interet(
            self.image_des_zones_a_explorer_moyennee_normalisee,
            seuil_min=0.3,
            max_points=100
        )

    def lecture_de_image(self, path: str) -> np.ndarray:
        """Charge l'image en niveaux de gris."""
        image_originelle = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if image_originelle is None:
            raise FileNotFoundError(f"Impossible de lire l'image : {path}")
        
        hauteur, largeur = image_originelle.shape
        self.dimensions_image = {"hauteur": hauteur, "largeur": largeur}
        
        return image_originelle
    
    def suppression_des_pixels_inutilisables_et_recherche_des_zones(
        self, image_changement_de_valeur: np.ndarray
    ) -> np.ndarray:
        """Supprime les pixels inutilisables et identifie les zones navigables."""
        image_sans_obstacles = self.suppression_des_zones_autour_des_obstacles(
            image_changement_de_valeur
        )
        image_finale = self.suppression_des_pixels_inconnus_non_accessibles(
            image_sans_obstacles
        )
        
        # Identification des zones navigables
        self.zones_connues, self.id_des_zones_pour_chaque_pixel = \
            self.identification_des_zones_aux_pixels_connus_accessibles(image_sans_obstacles)
        
        return image_finale

    def traitement_de_image_nettoyee(self, image: np.ndarray) -> np.ndarray:
        """Applique des filtres de lissage et normalise l'image."""
        # Double passage du filtre moyenneur pour un meilleur lissage
        image_moyennee_1 = self.application_du_filtre_moyenneur(image)
        image_moyennee_2 = self.application_du_filtre_moyenneur(image_moyennee_1)
        image_normalisee = self.normalisation_de_image(image_moyennee_2)
        
        return image_normalisee

    def metriques_physiques(self) -> None:
        """Définit les métriques physiques pour les filtres."""
        taille_pixel_en_realite = {"hauteur": 0.1, "largeur": 0.1}
        taille_robot_en_realite = {"hauteur": 0.3, "largeur": 0.3}
        
        self.taille_robot_en_pixel = {
            "hauteur": max(1, int(taille_robot_en_realite["hauteur"] / taille_pixel_en_realite["hauteur"])),
            "largeur": max(1, int(taille_robot_en_realite["largeur"] / taille_pixel_en_realite["largeur"]))
        }
        
        self.matrice_zone_robot = np.ones(
            (self.taille_robot_en_pixel["hauteur"], self.taille_robot_en_pixel["largeur"]), 
            dtype=np.float32
        )

    def changement_des_valeurs_de_l_image(self, image: np.ndarray) -> np.ndarray:
        """Transforme les valeurs de pixels en valeurs de travail."""
        img = image.astype(np.float32)
        
        self.valeur_pixel_occupe = {"image_origine": 128, "image_voulue": -100}
        self.valeur_pixel_connu = {"image_origine": 255, "image_voulue": 0}
        self.valeur_pixel_inconnu = {"image_origine": 0, "image_voulue": 100}
        
        # Vectorisation pour améliorer les performances
        img[image == self.valeur_pixel_occupe["image_origine"]] = self.valeur_pixel_occupe["image_voulue"]
        img[image == self.valeur_pixel_connu["image_origine"]] = self.valeur_pixel_connu["image_voulue"]
        img[image == self.valeur_pixel_inconnu["image_origine"]] = self.valeur_pixel_inconnu["image_voulue"]
        
        return img

    def suppression_des_zones_autour_des_obstacles(self, image: np.ndarray) -> np.ndarray:
        """Marque comme supprimés les pixels dans la zone d'emprise du robot autour des obstacles."""
        img = image.copy()
        
        # Normalisation du noyau
        filtre_moyenneur = self.matrice_zone_robot / (
            self.taille_robot_en_pixel["hauteur"] * self.taille_robot_en_pixel["largeur"]
        )
        
        # Masque des pixels occupés
        masque_pixels_occupes = (img == self.valeur_pixel_occupe["image_voulue"]).astype(np.float32)
        
        # Convolution optimisée avec cv2.filter2D
        image_zones_obstacles = cv2.filter2D(
            masque_pixels_occupes, 
            ddepth=-1, 
            kernel=filtre_moyenneur,
            borderType=cv2.BORDER_CONSTANT
        )
        
        # Marquage des zones à supprimer
        masque_suppression = (image_zones_obstacles != 0)
        img[masque_suppression] = self.valeur_pixel_supprime
        
        return img

    def suppression_des_pixels_inconnus_non_accessibles(self, image: np.ndarray) -> np.ndarray:
        """Supprime les pixels inconnus sans suffisamment de voisins connus."""
        img = image.copy()
        
        # Filtre pondéré pour compter les voisins (plus de poids au centre)
        filtre_comptabiliseur = np.array([
            [0.5, 0.5, 0.5, 0.5, 0.5],
            [0.5, 1.0, 1.0, 1.0, 0.5],
            [0.5, 1.0, 0.0, 1.0, 0.5],
            [0.5, 1.0, 1.0, 1.0, 0.5],
            [0.5, 0.5, 0.5, 0.5, 0.5],
        ], dtype=np.float32)
        
        # Masque des pixels connus
        masque_pixels_connus = (img == self.valeur_pixel_connu["image_voulue"]).astype(np.float32)
        
        # Comptage des voisins connus
        nombre_voisins_connus = cv2.filter2D(
            masque_pixels_connus, 
            ddepth=-1, 
            kernel=filtre_comptabiliseur,
            borderType=cv2.BORDER_CONSTANT
        )
        
        # Masque de présence suffisante de voisins
        masque_voisins_suffisants = (nombre_voisins_connus > 1.0)
        
        # Suppression des pixels isolés
        img[~masque_voisins_suffisants] = self.valeur_pixel_supprime
        
        # Pondération par densité de voisins pour les pixels conservés
        img[masque_voisins_suffisants] *= nombre_voisins_connus[masque_voisins_suffisants]
        
        return img

    def application_du_filtre_moyenneur(self, image: np.ndarray) -> np.ndarray:
        """Applique un filtre moyenneur 3x3 en ignorant les pixels supprimés."""
        img = image.copy()
        masque_pixels_supprimes = (img == self.valeur_pixel_supprime)
        
        # Préparation pour la convolution
        tmp = img.copy()
        tmp[masque_pixels_supprimes] = 0.0
        
        filtre_moyenneur = np.ones((3, 3), dtype=np.float32) / 9.0
        
        # Convolution optimisée
        image_moyenne = cv2.filter2D(
            tmp, 
            ddepth=-1, 
            kernel=filtre_moyenneur, 
            borderType=cv2.BORDER_CONSTANT
        )
        
        # Restauration des pixels supprimés
        image_moyenne[masque_pixels_supprimes] = self.valeur_pixel_supprime
        
        return image_moyenne

    def normalisation_de_image(self, image: np.ndarray) -> np.ndarray:
        """Normalise l'image entre 0 et 1, en excluant les pixels supprimés."""
        img = image.copy()
        masque_pixels_supprimes = (img == self.valeur_pixel_supprime)
        
        # Extraction des valeurs valides
        valeurs_valides = img[~masque_pixels_supprimes]
        
        if valeurs_valides.size == 0:
            return np.zeros_like(img, dtype=np.float32)
        
        minimum = float(np.min(valeurs_valides))
        maximum = float(np.max(valeurs_valides))
        
        # Gestion du cas constant
        if np.isclose(maximum, minimum):
            normalisee = np.zeros_like(img, dtype=np.float32)
            normalisee[masque_pixels_supprimes] = self.valeur_pixel_supprime
            return normalisee
        
        # Normalisation
        normalisee = (img - minimum) / (maximum - minimum)
        normalisee[masque_pixels_supprimes] = self.valeur_pixel_supprime
        
        return normalisee

    def identification_des_zones_aux_pixels_connus_accessibles(
        self, image: np.ndarray
    ) -> Tuple[Dict, np.ndarray]:
        """Identifie les zones connexes de pixels connus accessibles."""
        # Image binaire des pixels connus
        image_binaire = (image == self.valeur_pixel_connu["image_voulue"]).astype(np.uint8) * 255
        
        # Détection des composantes connexes (8-connectivité)
        nb_components, labels, stats, _ = cv2.connectedComponentsWithStats(
            image_binaire, 
            connectivity=8
        )
        
        zones = {}
        
        for label in range(1, nb_components):
            top = int(stats[label, cv2.CC_STAT_TOP])
            left = int(stats[label, cv2.CC_STAT_LEFT])
            height = int(stats[label, cv2.CC_STAT_HEIGHT])
            width = int(stats[label, cv2.CC_STAT_WIDTH])
            
            # Extraction du masque local
            zone_mask = (labels[top:top + height, left:left + width] == label).astype(np.uint8)
            
            zones[label] = {
                "mask": zone_mask,
                "offset": (left, top),
            }
        
        return zones, labels.astype(np.int32)

    def recuperation_zone_navigable_en_fonction_de_la_position_du_robot(
        self, x: int, y: int
    ) -> Optional[Dict]:
        """Retourne la zone navigable contenant la position (x, y)."""
        h, w = self.id_des_zones_pour_chaque_pixel.shape
        
        if not (0 <= x < w and 0 <= y < h):
            return None
        
        id_zone = int(self.id_des_zones_pour_chaque_pixel[y, x])
        return self.zones_connues.get(id_zone, None)
    
    def extraire_points_interet(
        self, 
        heatmap: np.ndarray, 
        seuil_min: float = 0.1, 
        max_points: Optional[int] = None
    ) -> List[Tuple[int, int, float]]:
        """Extrait les points d'intérêt d'une heatmap normalisée."""
        # Masque des pixels au-dessus du seuil
        mask = heatmap >= seuil_min
        
        if not np.any(mask):
            return []
        
        # Extraction des coordonnées et scores
        ys, xs = np.where(mask)
        scores = heatmap[ys, xs]
        
        # Construction efficace avec zip et conversion
        points = [(int(x), int(y), float(s)) for x, y, s in zip(xs, ys, scores)]
        
        # Tri décroissant par score
        points.sort(key=lambda t: t[2], reverse=True)
        
        # Limitation du nombre de points
        if max_points is not None:
            points = points[:max_points]
        
        return points
    
    def recuperer_les_n_points_interets(self) -> List[Tuple[int, int, float]]:
        """Retourne la liste des points d'intérêt."""
        return self.points_interet


class MultiRobotExploration:
    """Classe pour la planification multi-robots d'exploration."""
    
    def __init__(
        self, 
        heatmap: np.ndarray, 
        zones_connues: Dict, 
        id_zones_par_pixel: np.ndarray, 
        robot_positions: List[Tuple[int, int]]
    ):
        self.heatmap = heatmap
        self.zones_connues = zones_connues
        self.id_zones_par_pixel = id_zones_par_pixel
        self.robot_positions = robot_positions
        self.robot_targets = [None] * len(robot_positions)

    def get_zone_of_robot(self, robot_idx: int) -> Optional[Dict]:
        """Récupère la zone d'un robot donné."""
        x, y = self.robot_positions[robot_idx]
        h, w = self.id_zones_par_pixel.shape
        
        if not (0 <= x < w and 0 <= y < h):
            return None
        
        zone_id = int(self.id_zones_par_pixel[y, x])
        return self.zones_connues.get(zone_id, None)

    def get_points_of_interest(self, zone: Dict) -> List[Tuple[int, int, float]]:
        """Récupère les points d'intérêt dans une zone donnée."""
        mask = zone["mask"]
        offset_x, offset_y = zone["offset"]
        
        ys, xs = np.where(mask)
        
        # Calcul des coordonnées globales et extraction des scores
        coords_globales_x = offset_x + xs
        coords_globales_y = offset_y + ys
        scores = self.heatmap[coords_globales_y, coords_globales_x]
        
        # Filtrage et construction de la liste
        points = [
            (int(gx), int(gy), float(s)) 
            for gx, gy, s in zip(coords_globales_x, coords_globales_y, scores) 
            if s > 0
        ]
        
        # Tri décroissant par score
        points.sort(key=lambda t: t[2], reverse=True)
        
        return points

    def astar(
        self, 
        start: Tuple[int, int], 
        goal: Tuple[int, int], 
        occupancy_grid: np.ndarray
    ) -> Optional[List[Tuple[int, int]]]:
        """Pathfinding A* sur grille."""
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4-connectivité
        h, w = occupancy_grid.shape
        
        open_set = []
        heapq.heappush(open_set, (self.heuristic(start, goal), 0, start, [start]))
        visited = set()
        
        while open_set:
            f, g, current, path = heapq.heappop(open_set)
            
            if current in visited:
                continue
            
            visited.add(current)
            
            if current == goal:
                return path
            
            for dx, dy in neighbors:
                nx, ny = current[0] + dx, current[1] + dy
                
                if 0 <= nx < w and 0 <= ny < h and occupancy_grid[ny, nx] == 0:
                    if (nx, ny) not in visited:
                        new_g = g + 1
                        new_f = new_g + self.heuristic((nx, ny), goal)
                        heapq.heappush(open_set, (new_f, new_g, (nx, ny), path + [(nx, ny)]))
        
        return None  # Pas de chemin trouvé

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> int:
        """Distance de Manhattan."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def choose_targets(self, occupancy_grid: np.ndarray) -> List[Optional[Tuple[int, int]]]:
        """Choisit le point d'intérêt optimal pour chaque robot."""
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
                path = self.astar(pos, (px, py), occupancy_grid)
                
                if path is not None and len(path) < best_distance:
                    best_distance = len(path)
                    best_point = (px, py)
            
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