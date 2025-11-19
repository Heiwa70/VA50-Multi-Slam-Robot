import numpy as np
import cv2

class Detection_points_interets:

    def __init__(self, path):

        image_originelle = self.lecture_de_image(path)

        # valeur sentinelle pour "pixel supprimé" (entier)
        self.valeur_pixel_supprime = -999999

        self.metriques_physiques()

        image_changement_de_valeur = self.changement_des_valeurs_de_l_image(image_originelle)

        image_avec_pixels_connus_accessibles_et_inconnus_accessibles = self.suppression_des_pixels_innutilisable_et_recherche_des_zones(image_changement_de_valeur)
        image_des_zones_a_explorer_moyennee_normalisee = self.traitement_de_image_netoyee(self, image_avec_pixels_connus_accessibles_et_inconnus_accessibles)
        
        self.points_interet = self.extraire_points_interet(
            image_des_zones_a_explorer_moyennee_normalisee,
            seuil_min=0.3,    # on garde seulement les zones vraiment attractives
            max_points=100     # on limite à 100 points les plus intéressants
        )

    def lecture_de_image(self, path):

        image_originelle = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if image_originelle is None:
            raise FileNotFoundError(f"Impossible de lire l'image : {path}")

        hauteur, largeur = image_originelle.shape
        self.dimensions_image = {"hauteur": hauteur, "largeur": largeur}

        return image_originelle
    
    def suppression_des_pixels_innutilisable_et_recherche_des_zones(self, image_changement_de_valeur):

        image_avec_pixels_connus_accessibles_et_inconnus = self.suppression_des_zones_autours_des_obstacles(image_changement_de_valeur)
        image_avec_pixels_connus_accessibles_et_inconnus_accessibles = self.suppression_des_pixels_inconnus_non_accessibles(image_avec_pixels_connus_accessibles_et_inconnus)

        # identification des zones (on envoie l'image contenant les pixels "connus" marqués par valeur_pixel_connu["image_voulue"])
        self.zones_connus, self.id_des_zones_pour_chaque_pixel = self.identification_des_zones_aux_pixels_connus_accessibles(image_avec_pixels_connus_accessibles_et_inconnus)

        return image_avec_pixels_connus_accessibles_et_inconnus_accessibles

    def traitement_de_image_netoyee(self, image_avec_pixels_connus_accessibles_et_inconnus_accessibles):

        image_des_zones_a_explorer = self.application_du_filtre_moyenneur(image_avec_pixels_connus_accessibles_et_inconnus_accessibles)
        image_des_zones_a_explorer_moyennee = self.application_du_filtre_moyenneur(image_des_zones_a_explorer)
        image_des_zones_a_explorer_moyennee_normalisee = self.normalisation_de_image(image_des_zones_a_explorer_moyennee)

        return image_des_zones_a_explorer_moyennee_normalisee

    def metriques_physiques(self):
        ###### Metriques pour les filtres ######
        taille_pixel_en_realite = {"hauteur": 0.1, "largueur": 0.1}
        taille_robot_en_realite = {"hauteur": 0.3, "largueur": 0.3}

        self.taille_robot_en_pixel = {
            "hauteur": max(1, int(taille_robot_en_realite["hauteur"] / taille_pixel_en_realite["hauteur"])),
            "largueur": max(1, int(taille_robot_en_realite["largueur"] / taille_pixel_en_realite["largueur"]))
        }

        self.matrice_zone_robot = np.ones((self.taille_robot_en_pixel["hauteur"], self.taille_robot_en_pixel["largueur"]), dtype=np.float32)


    def changement_des_valeurs_de_l_image(self, image):
        ###### Changement de valeurs (entiers ou floats autorisés) ######
        img = image.copy().astype(np.float32)

        self.valeur_pixel_occupe = {"image_origine": 128, "image_voulue": -100}
        self.valeur_pixel_connu = {"image_origine": 255, "image_voulue": 0}
        self.valeur_pixel_inconnu = {"image_origine": 0, "image_voulue": 100}

        img[image == self.valeur_pixel_occupe["image_origine"]] = self.valeur_pixel_occupe["image_voulue"]
        img[image == self.valeur_pixel_connu["image_origine"]] = self.valeur_pixel_connu["image_voulue"]
        img[image == self.valeur_pixel_inconnu["image_origine"]] = self.valeur_pixel_inconnu["image_voulue"]

        return img


    def suppression_des_zones_autours_des_obstacles(self, image):
        """
        Marque comme 'supprimés' (sentinelle) les pixels se trouvant dans la zone
        d'emprise du robot autour d'obstacles occupés.
        """

        img = image.copy().astype(np.float32)

        # noyau moyenneur = zone du robot normalisée
        filtre_moyenneur_autour_robot = self.matrice_zone_robot / (self.taille_robot_en_pixel["hauteur"] * self.taille_robot_en_pixel["largueur"])
        filtre_moyenneur_autour_robot = filtre_moyenneur_autour_robot.astype(np.float32)

        # masque des pixels occupés (valeur ciblée dans l'image transformée)
        masque_pixels_occupes = (img == self.valeur_pixel_occupe["image_voulue"]).astype(np.float32)

        # utiliser cv2.filter2D (plus rapide que convolve2d)
        image_avec_zones_autour_des_pixels_occupes = cv2.filter2D(masque_pixels_occupes, ddepth=-1, kernel=filtre_moyenneur_autour_robot,
                                                                  borderType=cv2.BORDER_CONSTANT)

        masque_des_zones_autour_des_pixels_occupes = (image_avec_zones_autour_des_pixels_occupes != 0)
        img[masque_des_zones_autour_des_pixels_occupes] = self.valeur_pixel_supprime

        return img


    def suppression_des_pixels_inconnus_non_accessibles(self, image):
        """
        Supprime (marque) les pixels 'inconnus' qui n'ont pas suffisamment de pixels 'connus'
        dans leur voisinage. Multiplie ensuite les pixels valides par le score local.
        """

        img = image.copy().astype(np.float32)

        filtre_comptabiliseur = np.array([
            [0.5, 0.5, 0.5, 0.5, 0.5],
            [0.5, 1.0, 1.0, 1.0, 0.5],
            [0.5, 1.0, 0.0, 1.0, 0.5],
            [0.5, 1.0, 1.0, 1.0, 0.5],
            [0.5, 0.5, 0.5, 0.5, 0.5],
        ], dtype=np.float32)

        masque_des_pixels_connus = (img == self.valeur_pixel_connu["image_voulue"]).astype(np.float32)

        nombres_de_pixels_connus_aux_alentours = cv2.filter2D(masque_des_pixels_connus, ddepth=-1, kernel=filtre_comptabiliseur,
                                                             borderType=cv2.BORDER_CONSTANT)

        masque_presence_des_pixels_connus_aux_alentours = (nombres_de_pixels_connus_aux_alentours > 1.0)

        # marquer comme supprimés les pixels qui n'ont pas assez de voisins connus
        img[~masque_presence_des_pixels_connus_aux_alentours] = self.valeur_pixel_supprime

        # pour les pixels conservés, multiplier par le score local (concentration)
        img[masque_presence_des_pixels_connus_aux_alentours] *= nombres_de_pixels_connus_aux_alentours[masque_presence_des_pixels_connus_aux_alentours]

        return img


    def application_du_filtre_moyenneur(self, image):
        """
        Applique un filtre moyenneur 3x3 en ignorant les pixels marqués 'supprimés'.
        Les pixels supprimés restent marqués après convolution.
        """

        img = image.copy().astype(np.float32)
        masque_pixels_supprimes = (img == self.valeur_pixel_supprime)

        tmp = img.copy()
        tmp[masque_pixels_supprimes] = 0.0  # exclure supprimés du calcul

        filtre_moyenneur = np.ones((3, 3), dtype=np.float32) / 9.0

        # on remplace convolve2d par filter2D (bord constant -> valeur 0)
        image_moyenne = cv2.filter2D(tmp, ddepth=-1, kernel=filtre_moyenneur, borderType=cv2.BORDER_CONSTANT)

        # restauration des pixels supprimés
        image_moyenne[masque_pixels_supprimes] = self.valeur_pixel_supprime

        return image_moyenne


    def normalisation_de_image(self, image):
        """
        Normalise entre 0 et 1 en excluant les pixels marqués supprimés.
        """

        img = image.copy().astype(np.float32)
        masque_pixels_supprimes = (img == self.valeur_pixel_supprime)

        # exclure les pixels supprimés pour le min/max
        valeurs_valides = img[~masque_pixels_supprimes]
        if valeurs_valides.size == 0:
            return np.zeros_like(img, dtype=np.float32)

        minimum = float(np.min(valeurs_valides))
        maximum = float(np.max(valeurs_valides))

        if maximum == minimum:
            # tout constant -> renvoyer zéros (hors pixels supprimés)
            normalisee = np.zeros_like(img, dtype=np.float32)
            normalisee[masque_pixels_supprimes] = self.valeur_pixel_supprime
            return normalisee

        normalisee = (img - minimum) * (1.0 / (maximum - minimum))
        normalisee[masque_pixels_supprimes] = self.valeur_pixel_supprime
        return normalisee


    def identification_des_zones_aux_pixels_connus_accessibles(self, image):
        """
        Retourne :
            - zones : dict[label] = {"mask": mask_local (0/1), "offset": (x, y)}
            - label_image : image 2D (int) où chaque pixel a l'id de la zone (0 = fond)
        """

        # construire image binaire (True pour pixel connu)
        image_binaire = (image == self.valeur_pixel_connu["image_voulue"])

        # connectedComponents attend uint8 (0 ou 255)
        img_uint8 = (image_binaire.astype(np.uint8) * 255)

        # 1) Détection des composantes connexes
        nb_components, labels, stats, _ = cv2.connectedComponentsWithStats(img_uint8, connectivity=8)

        zones = {}

        for label in range(1, nb_components):
            # top = row (y), left = col (x)
            top = int(stats[label, cv2.CC_STAT_TOP])
            left = int(stats[label, cv2.CC_STAT_LEFT])
            height = int(stats[label, cv2.CC_STAT_HEIGHT])
            width = int(stats[label, cv2.CC_STAT_WIDTH])

            # masque local dans l'espace (rows,cols) -> labels[y:y+h, x:x+w]
            zone_mask = (labels[top:top + height, left:left + width] == label).astype(np.uint8)

            # offset (x, y) = (col, row)
            zones[label] = {
                "mask": zone_mask,
                "offset": (left, top),
            }

        label_image = labels.copy().astype(np.int32)
        return zones, label_image


    def recuperation_zone_navigable_en_fonction_de_la_position_du_robot(self, x, y):
        """
        x = colonne, y = ligne
        renvoie la zone (dict) ou None
        """
        # safety : vérifier limites
        h = self.id_des_zones_pour_chaque_pixel.shape[0]
        w = self.id_des_zones_pour_chaque_pixel.shape[1]
        if not (0 <= x < w and 0 <= y < h):
            return None

        id_de_la_zone = int(self.id_des_zones_pour_chaque_pixel[y, x])
        return self.zones_connus.get(id_de_la_zone, None)
    
    def extraire_points_interet(self, heatmap, seuil_min=0.1, max_points=None):
        """
        Extrait les points d'intérêt d'une heatmap normalisée (0-1).

        Args:
            heatmap (np.ndarray): image normalisée des zones à explorer
            seuil_min (float): score minimum pour considérer un point comme intéressant
            max_points (int | None): nombre maximum de points à retourner (top-K)

        Returns:
            List[Tuple[int,int,float]]: liste de tuples (x, y, score), triée par score décroissant
        """

        # Masque des pixels au-dessus du seuil
        mask = heatmap >= seuil_min

        if not np.any(mask):
            return []

        # Récupérer les coordonnées et les scores
        ys, xs = np.where(mask)
        scores = heatmap[ys, xs]

        # Construire la liste des tuples
        points = list(zip(xs, ys, scores))

        # Tri décroissant par score
        points.sort(key=lambda t: t[2], reverse=True)

        # Limiter le nombre de points si demandé
        if max_points is not None:
            points = points[:max_points]

        return points
    
    def recuperer_les_n_points_interets(self):

        return self.points_interet

if "__main__" == __name__:
    path_image = "mon image.png"
    detection_des_points_d_interets = Detection_points_interets(path_image)
